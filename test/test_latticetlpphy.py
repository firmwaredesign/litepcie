#
# This file is part of LitePCIe.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *
from migen.sim import run_simulation, passive

from litex.gen import *

from litex.soc.interconnect import wishbone

from litepcie.gen                import get_pcie_ios_lattice_tlp
from litepcie.phy.latticetlpphy  import LatticeTLPPHY
from litepcie.core               import LitePCIeEndpoint, LitePCIeMSI
from litepcie.core.msi           import LitePCIeMSITLP
from litepcie.frontend.wishbone  import LitePCIeWishboneMaster

# Helpers ------------------------------------------------------------------------------------------

class Pads:
    # Pads with the same names/widths as the ones generated for the standalone core.
    def __init__(self):
        (_, _, *subsignals), = get_pcie_ios_lattice_tlp()
        for s in subsignals:
            setattr(self, s.name, Signal(len(s.constraints[0].identifiers), name=s.name))

def lattice_word(tlp_bytes):
    # Lattice vc_rx/vc_tx data: TLP byte 0 on bits 7:0 (see Lattice pcie_rx_engine.v/pcie_tx_engine.v).
    return int.from_bytes(bytes(tlp_bytes), "little")

def lattice_words(tlp_bytes):
    return [lattice_word(tlp_bytes[i:i+4]) for i in range(0, len(tlp_bytes), 4)]

ENDPOINT_ID  = 0x0300
REQUESTER_ID = 0x0100
MSI_ADDRESS  = 0xfee0_1000
MSI_DATA     = 0x4020
CSR_VALUE    = 0x1234_5678

class DUT(LiteXModule):
    def __init__(self, with_msi=True, with_master=False, **phy_kwargs):
        self.pads = pads = Pads()
        self.phy  = phy  = LatticeTLPPHY(None, pads, data_width=64, pcie_data_width=32, bar0_size=0x40000, **phy_kwargs)
        self.endpoint = endpoint = LitePCIeEndpoint(phy, endianness=phy.endianness, address_width=32)

        # BAR0: Wishbone SRAM.
        self.wb_master = LitePCIeWishboneMaster(endpoint)
        self.sram      = wishbone.SRAM(64, init=[CSR_VALUE])
        self.comb += self.wb_master.wishbone.connect(self.sram.bus)

        # MSI over TLP.
        self.msi_tlp = LitePCIeMSITLP(endpoint,
            enable  = phy.msi_enable,
            address = phy.msi_address,
            data    = phy.msi_data,
            mme     = phy.msi_mme,
            mask    = phy.msi_mask,
        )
        if with_master:
            self.master = endpoint.crossbar.get_master_port()
        if with_msi:
            self.msi = LitePCIeMSI(width=32)
            self.comb += self.msi.source.connect(self.msi_tlp.sink)

def run(test, stimulus, **kwargs):
    dut = DUT(**kwargs)
    tlps = []

    @passive
    def tx_monitor():
        words = []
        yield dut.pads.tx_ready.eq(1)
        while True:
            if (yield dut.pads.tx_valid) and (yield dut.pads.tx_ready):
                data = (yield dut.pads.tx_data)
                datap = (yield dut.pads.tx_datap)
                for i in range(4):
                    test.assertEqual((datap >> i) & 1, bin((data >> 8*i) & 0xff).count("1") & 1)
                test.assertEqual((yield dut.pads.tx_sop), len(words) == 0)
                test.assertEqual((yield dut.pads.tx_eop_n), 0)
                words.append(data)
                if (yield dut.pads.tx_eop):
                    tlps.append(words)
                    words = []
            yield

    def configure():
        yield dut.pads.rst_n.eq(1)
        yield dut.pads.id.eq(ENDPOINT_ID)
        yield dut.pads.msi_address.eq(MSI_ADDRESS)
        yield dut.pads.msi_data.eq(MSI_DATA)
        if hasattr(dut, "msi"):
            yield dut.msi.enable.storage.eq(0xffffffff)
        for _ in range(8):
            yield

    def generator():
        yield from configure()
        yield from stimulus(dut, tlps)

    run_simulation(dut, [generator(), tx_monitor()], clocks={"sys": 10, "pcie": 10})

def monitor_counts(dut):
    # CSR fields are only combined into .status in a CSR bank: read the fields.
    fields = dut.phy.tlp_monitor._counts.fields
    counts = {}
    for name in ["tx_requests", "tx_completions", "rx_requests", "rx_completions"]:
        counts[name] = (yield getattr(fields, name))
    return counts

def wait(n=64):
    for _ in range(n):
        yield

def send_rx_tlp(dut, words):
    for i, word in enumerate(words):
        yield dut.pads.rx_valid.eq(1)
        yield dut.pads.rx_sop.eq(i == 0)
        yield dut.pads.rx_eop.eq(i == len(words) - 1)
        yield dut.pads.rx_data.eq(word)
        yield
        while not (yield dut.pads.rx_ready):
            yield
    yield dut.pads.rx_valid.eq(0)

def pulse_irq(dut, n=0):
    yield dut.msi.irqs.eq(1 << n)
    yield
    yield dut.msi.irqs.eq(0)

MSI_TLP = lattice_words([
    0x40, 0x00, 0x00, 0x01,                        # MWr32, Length 1.
    ENDPOINT_ID >> 8, ENDPOINT_ID & 0xff, 0x00, 0x0f, # Requester ID, Tag 0, Last BE 0/First BE f.
    0xfe, 0xe0, 0x10, 0x00,                        # Address.
    MSI_DATA & 0xff, MSI_DATA >> 8, 0x00, 0x00,    # Message Data (little-endian payload).
])

# Tests --------------------------------------------------------------------------------------------

class TestLatticeTLPPHY(unittest.TestCase):
    def test_bar0_read_completion(self):
        def stimulus(dut, tlps):
            yield dut.pads.bus_master_enable.eq(1)
            mrd = lattice_words([
                0x00, 0x00, 0x00, 0x01,                                # MRd32, Length 1.
                REQUESTER_ID >> 8, REQUESTER_ID & 0xff, 0x12, 0x0f,    # Requester ID, Tag, BEs.
                0x00, 0x00, 0x00, 0x00,                                # Address.
            ])
            yield from send_rx_tlp(dut, mrd)
            yield from wait()
            self.assertEqual(tlps, [lattice_words([
                0x4a, 0x00, 0x00, 0x01,                             # CplD, Length 1.
                ENDPOINT_ID >> 8, ENDPOINT_ID & 0xff, 0x00, 0x04,   # Completer ID, Byte Count 4.
                REQUESTER_ID >> 8, REQUESTER_ID & 0xff, 0x12, 0x00, # Requester ID, Tag, Lower Address.
                *CSR_VALUE.to_bytes(4, "little"),                   # Data (little-endian payload).
            ])])
            self.assertEqual((yield from monitor_counts(dut)),
                {"tx_requests": 0, "tx_completions": 1, "rx_requests": 1, "rx_completions": 0})
        run(self, stimulus)

    def test_msi(self):
        def stimulus(dut, tlps):
            yield dut.pads.bus_master_enable.eq(1)
            yield dut.pads.msi_enable.eq(1)
            yield from wait(8)
            yield from pulse_irq(dut)
            yield from wait()
            self.assertEqual(tlps, [MSI_TLP])
            self.assertEqual((yield from monitor_counts(dut))["tx_requests"], 1)
            mon = dut.phy.tlp_monitor
            self.assertEqual([(yield mon._tx_request_header0.status), (yield mon._tx_request_header1.status),
                (yield mon._tx_request_header2.status)], MSI_TLP[:3])
        run(self, stimulus)

    def test_msi_held_while_disabled_or_masked(self):
        def stimulus(dut, tlps):
            # MSI Enable cleared.
            yield dut.pads.bus_master_enable.eq(1)
            yield from pulse_irq(dut)
            yield from wait()
            self.assertEqual(tlps, [])
            yield dut.pads.msi_enable.eq(1)
            yield from wait()
            self.assertEqual(tlps, [MSI_TLP])

            # Vector 0 masked.
            yield dut.pads.msi_mask.eq(1)
            yield from pulse_irq(dut)
            yield from wait()
            self.assertEqual(tlps, [MSI_TLP])
            yield dut.pads.msi_mask.eq(0)
            yield from wait()
            self.assertEqual(tlps, [MSI_TLP, MSI_TLP])
        run(self, stimulus)

    def test_requests_held_without_bus_master_enable(self):
        def stimulus(dut, tlps):
            yield dut.pads.msi_enable.eq(1)
            yield from pulse_irq(dut)
            yield from wait()
            self.assertEqual(tlps, [])
            yield dut.pads.bus_master_enable.eq(1)
            yield from wait()
            self.assertEqual(tlps, [MSI_TLP])
        run(self, stimulus)

    def test_msi_multi_message_vector(self):
        def stimulus(dut, tlps):
            yield dut.pads.bus_master_enable.eq(1)
            yield dut.pads.msi_enable.eq(1)
            yield dut.pads.msi_mme.eq(2) # 4 vectors: vector in the 2 LSBs of Message Data.
            yield from wait(8)
            # Drive the vector directly (LitePCIeMSI is single-vector).
            yield dut.msi_tlp.sink.valid.eq(1)
            yield dut.msi_tlp.sink.dat.eq(3)
            yield
            while not (yield dut.msi_tlp.sink.ready):
                yield
            yield dut.msi_tlp.sink.valid.eq(0)
            yield from wait()
            self.assertEqual(len(tlps), 1)
            self.assertEqual(tlps[0][3], (MSI_DATA & ~0x3) | 3)
        run(self, stimulus, with_msi=False)

    def test_rx_trash_dword_dropped(self):
        # The IP may append a trash DWORD to received TLPs: trimmed to the header length.
        def stimulus(dut, tlps):
            yield dut.pads.bus_master_enable.eq(1)
            yield from send_rx_tlp(dut, lattice_words([
                0x00, 0x00, 0x00, 0x01,                                # MRd32, Length 1.
                REQUESTER_ID >> 8, REQUESTER_ID & 0xff, 0x12, 0x0f,
                0x00, 0x00, 0x00, 0x00,
                0xde, 0xad, 0xbe, 0xef,                                # Trash DWORD.
            ]))
            yield from send_rx_tlp(dut, lattice_words([
                0x00, 0x00, 0x00, 0x01,                                # MRd32, Length 1 (no trash).
                REQUESTER_ID >> 8, REQUESTER_ID & 0xff, 0x13, 0x0f,
                0x00, 0x00, 0x00, 0x00,
            ]))
            yield from wait(128)
            self.assertEqual([(tlp[2] >> 16) & 0xff for tlp in tlps], [0x12, 0x13]) # Tags.
            fields = dut.phy.tlp_monitor._rx_dwords.fields
            self.assertEqual((yield fields.dropped), 1)
            self.assertEqual((yield fields.last_tlp), 3)
        run(self, stimulus)

    def test_fpga_read_request(self):
        # MRd32 is 3 DWORDs: the padding of the last 64-bit beat must not reach the IP.
        def stimulus(dut, tlps):
            yield dut.pads.bus_master_enable.eq(1)
            port = dut.master
            yield port.source.channel.eq(port.channel)
            yield port.source.first.eq(1)
            yield port.source.last.eq(1)
            yield port.source.we.eq(0)
            yield port.source.adr.eq(0x1234_5678)
            yield port.source.len.eq(1)
            yield port.source.req_id.eq(ENDPOINT_ID)
            yield port.source.valid.eq(1)
            yield
            while not (yield port.source.ready):
                yield
            yield port.source.valid.eq(0)
            yield from wait()
            self.assertEqual(len(tlps), 1)
            self.assertEqual(tlps[0][0] & 0xff, 0x00) # MRd32.
            self.assertEqual(tlps[0][2], lattice_word([0x12, 0x34, 0x56, 0x78]))
            self.assertEqual(len(tlps[0]), 3)
            tag = (tlps[0][1] >> 16) & 0xff

            # Odd-length Completion (3 DWORDs header + 2 DWORDs data) back from the Host: the
            # up-converted beat must not carry a stale DWORD.
            completions = []
            yield from send_rx_tlp(dut, lattice_words([
                0x4a, 0x00, 0x00, 0x02,                             # CplD, Length 2.
                REQUESTER_ID >> 8, REQUESTER_ID & 0xff, 0x00, 0x08, # Completer ID, Byte Count 8.
                ENDPOINT_ID >> 8, ENDPOINT_ID & 0xff, tag, 0x78,    # Requester ID, Tag, Lower Address.
                0x11, 0x22, 0x33, 0x44,
                0x55, 0x66, 0x77, 0x88,
                0xde, 0xad, 0xbe, 0xef, # Trash DWORD appended by the IP.
            ]))
            for _ in range(64):
                if (yield port.sink.valid):
                    completions.append(((yield port.sink.dat), (yield port.sink.len), (yield port.sink.last)))
                    yield port.sink.ready.eq(1)
                    yield
                    yield port.sink.ready.eq(0)
                yield
            self.assertEqual(len(completions), 1)
            self.assertEqual((yield from monitor_counts(dut)),
                {"tx_requests": 1, "tx_completions": 0, "rx_requests": 0, "rx_completions": 1})
            mon = dut.phy.tlp_monitor
            self.assertEqual((yield mon._rx_completion_header2.status),
                lattice_word([ENDPOINT_ID >> 8, ENDPOINT_ID & 0xff, tag, 0x78]))
            dat, length, last = completions[0]
            self.assertEqual(length, 2)
            self.assertEqual(last, 1)
            self.assertEqual({dat & 0xffffffff, dat >> 32}, {lattice_word([0x11, 0x22, 0x33, 0x44]), lattice_word([0x55, 0x66, 0x77, 0x88])})
        run(self, stimulus, with_master=True)

    def test_fpga_write_request_odd_length(self):
        # MWr32 with 2 DWORDs of payload is 5 DWORDs.
        def stimulus(dut, tlps):
            yield dut.pads.bus_master_enable.eq(1)
            port = dut.master
            yield port.source.channel.eq(port.channel)
            yield port.source.first.eq(1)
            yield port.source.last.eq(1)
            yield port.source.we.eq(1)
            yield port.source.adr.eq(0x1234_5678)
            yield port.source.len.eq(2)
            yield port.source.req_id.eq(ENDPOINT_ID)
            yield port.source.dat.eq(0x1111_1111_2222_2222)
            yield port.source.valid.eq(1)
            yield
            while not (yield port.source.ready):
                yield
            yield port.source.valid.eq(0)
            yield from wait()
            self.assertEqual(len(tlps), 1)
            self.assertEqual(tlps[0][0] & 0xff, 0x40) # MWr32.
            self.assertEqual(len(tlps[0]), 5)
        run(self, stimulus, with_master=True)

    def check_max_sizes(self, cases, **phy_kwargs):
        def stimulus(dut, tlps):
            for mps, mrrs, exp_mps, exp_mrrs in cases:
                yield dut.pads.max_payload_size.eq(mps)
                yield dut.pads.max_read_request_size.eq(mrrs)
                yield
                yield
                self.assertEqual((yield dut.phy.max_payload_size), exp_mps)
                self.assertEqual((yield dut.phy.max_request_size), exp_mrrs)
        run(self, stimulus, **phy_kwargs)

    def test_max_sizes_decode_default(self):
        # Limited to 128-byte TLPs by default, whatever the Host negotiated.
        self.check_max_sizes([(0, 0, 128, 128), (1, 2, 128, 128), (5, 5, 128, 128)])

    def test_max_sizes_decode(self):
        self.check_max_sizes([
            (0, 0, 128, 128),
            (1, 1, 256, 256),
            (2, 2, 256, 512), # Payload clamped to IP's 256, Read Request to LitePCIe's 512.
            (5, 5, 256, 512),
        ], max_tlp_payload_size=4096)

if __name__ == "__main__":
    unittest.main()
