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

from litepcie.phy.lfd2nxpciephy import LFD2NXPCIEPHY
from litepcie.core              import LitePCIeEndpoint
from litepcie.frontend.wishbone import LitePCIeWishboneMaster

from test.test_latticetlpphy import lattice_word, lattice_words, ENDPOINT_ID, REQUESTER_ID, CSR_VALUE

# Helpers ------------------------------------------------------------------------------------------

class DummyPlatform:
    def request(self, name):
        return Signal(name=name)

class DummyPads:
    def __init__(self):
        for name in ["clk_p", "clk_n", "rx_p", "rx_n", "tx_p", "tx_n", "perst", "refret", "rext"]:
            setattr(self, name, Signal(name=name))

class SimLFD2NXPCIEPHY(LFD2NXPCIEPHY):
    # Simulate up to the IP's vc_rx_*/vc_tx_* (m_axis_rx/s_axis_tx): no hard IP/GSR Instance.
    def do_finalize(self):
        pass

class DUT(LiteXModule):
    def __init__(self):
        self.phy      = phy      = SimLFD2NXPCIEPHY(DummyPlatform(), DummyPads(), data_width=64, pcie_data_width=32, bar0_size=0x40000)
        self.endpoint = endpoint = LitePCIeEndpoint(phy, endianness=phy.endianness, address_width=32)
        self.comb += phy.id.eq(ENDPOINT_ID)

        self.wb_master = LitePCIeWishboneMaster(endpoint)
        self.sram      = wishbone.SRAM(64, init=[CSR_VALUE])
        self.comb += self.wb_master.wishbone.connect(self.sram.bus)

        self.master = endpoint.crossbar.get_master_port()

def run(test, stimulus):
    dut  = DUT()
    tlps = []

    @passive
    def tx_monitor():
        tx    = dut.phy.s_axis_tx
        words = []
        yield tx.ready.eq(1)
        while True:
            if (yield tx.valid) and (yield tx.ready):
                test.assertEqual((yield tx.first), len(words) == 0)
                words.append((yield tx.dat))
                if (yield tx.last):
                    tlps.append(words)
                    words = []
            yield

    run_simulation(dut, [stimulus(dut, tlps), tx_monitor()], clocks={"sys": 10, "pcie": 10})

def send_rx_tlp(dut, words):
    rx = dut.phy.m_axis_rx
    for i, word in enumerate(words):
        yield rx.valid.eq(1)
        yield dut.phy.ip_params["o_vc_rx_sop_o"].eq(i == 0) # Drives rx.first.
        yield rx.last.eq(i == len(words) - 1)
        yield rx.dat.eq(word)
        yield
        while not (yield rx.ready):
            yield
    yield rx.valid.eq(0)

def wait(n=64):
    for _ in range(n):
        yield

# Tests --------------------------------------------------------------------------------------------

class TestLFD2NXPCIEPHY(unittest.TestCase):
    def test_bar0_read_completion(self):
        def stimulus(dut, tlps):
            yield from wait(8)
            yield from send_rx_tlp(dut, lattice_words([
                0x00, 0x00, 0x00, 0x01,
                REQUESTER_ID >> 8, REQUESTER_ID & 0xff, 0x12, 0x0f,
                0x00, 0x00, 0x00, 0x00,
            ]))
            yield from wait()
            self.assertEqual(tlps, [lattice_words([
                0x4a, 0x00, 0x00, 0x01,
                ENDPOINT_ID >> 8, ENDPOINT_ID & 0xff, 0x00, 0x04,
                REQUESTER_ID >> 8, REQUESTER_ID & 0xff, 0x12, 0x00,
                *CSR_VALUE.to_bytes(4, "little"),
            ])])
        run(self, stimulus)

    def test_fpga_read_request_and_odd_length_completion(self):
        def stimulus(dut, tlps):
            yield from wait(8)
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

            # MRd32 is 3 DWORDs: no padding DWORD sent to the IP.
            self.assertEqual(len(tlps), 1)
            self.assertEqual(len(tlps[0]), 3)
            self.assertEqual(tlps[0][2], lattice_word([0x12, 0x34, 0x56, 0x78]))
            tag = (tlps[0][1] >> 16) & 0xff

            # 5-DWORD Completion back: one beat, no stale DWORD.
            yield from send_rx_tlp(dut, lattice_words([
                0x4a, 0x00, 0x00, 0x02,
                REQUESTER_ID >> 8, REQUESTER_ID & 0xff, 0x00, 0x08,
                ENDPOINT_ID >> 8, ENDPOINT_ID & 0xff, tag, 0x78,
                0x11, 0x22, 0x33, 0x44,
                0x55, 0x66, 0x77, 0x88,
            ]))
            completions = []
            for _ in range(64):
                if (yield port.sink.valid):
                    completions.append(((yield port.sink.dat), (yield port.sink.len), (yield port.sink.last)))
                    yield port.sink.ready.eq(1)
                    yield
                    yield port.sink.ready.eq(0)
                yield
            self.assertEqual(len(completions), 1)
            dat, length, last = completions[0]
            self.assertEqual((length, last), (2, 1))
            self.assertEqual({dat & 0xffffffff, dat >> 32}, {lattice_word([0x11, 0x22, 0x33, 0x44]), lattice_word([0x55, 0x66, 0x77, 0x88])})
        run(self, stimulus)

if __name__ == "__main__":
    unittest.main()
