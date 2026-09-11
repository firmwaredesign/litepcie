#
# This file is part of LitePCIe.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

# PHY for a Lattice PCIe IP instantiated OUTSIDE of LitePCIe (in the User top level), connected
# at the TLP level. Unlike LFD2NXPCIEPHY/LFCPNXPCIEPHY, nothing vendor-specific is instantiated
# here: the core only connects to the IP's TLP interface (vc_rx_*/vc_tx_*, same signals/semantics
# as the ones LFD2NXPCIEPHY wires internally), and gets its clock, link status and the
# configuration-space values it needs as inputs:
#
# - clk: the clock fed to the IP's clk_usr_i (not clk_usr_o: see below).
# - rst_n/*_link_up: from the IP (u_*_link_up_o), rst_n chosen by the User.
# - id: Bus/Device/Function, used as Requester/Completer ID. Not part of the standard
#   configuration header; read over ucfg at address 0x5F (Vendor Specific Capability).
# - bus_master_enable, max_payload_size, max_read_request_size, link_speed/width, msi_*: read
#   over ucfg from the configuration space (Command, Device Control, Link Status and MSI
#   capability registers).
#
# Don't clock the core or the IP's usr_lmmi_clk_i from clk_usr_o: with usr_lmmi_clk_i fed back
# from clk_usr_o, the IP doesn't come up and the device doesn't enumerate (seen on hardware).
#
# The IP has no MSI request interface, so MSIs are sent as Memory Write TLPs by LitePCIeMSITLP
# from msi_enable/msi_mme/msi_address/msi_data/msi_mask (see litepcie/core/msi.py).
#
# All status inputs are quasi-static and synchronous to clk (= "pcie" domain here).

from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *

from litepcie.common import *
from litepcie.tlp.common import *
from litepcie.tlp.common import max_payload_size as max_payload_size_limit
from litepcie.tlp.common import max_request_size as max_request_size_limit
from litepcie.phy.common import *

# LatticeTLPMonitor --------------------------------------------------------------------------------

class LatticeTLPMonitor(LiteXModule):
    """Bring-up/debug: count TLPs and capture headers at the IP's TLP interface.

    Counts Memory Requests (MRd/MWr, TLP Type 0, incl. MSIs) and Completions (Cpl/CplD) in each
    direction, and captures the 3 first DWORDs (raw vc_* DWORDs, TLP byte 0 on bits 7:0) of the last
    Memory Request sent to the Host and of the last Completion received from the Host.
    """
    def __init__(self, tx, rx, rx_dropped=0):
        self._counts = CSRStatus(fields=[
            CSRField("tx_requests",    size=8, description="Memory Requests sent (incl. MSIs), wraps."),
            CSRField("tx_completions", size=8, description="Completions sent, wraps."),
            CSRField("rx_requests",    size=8, description="Memory Requests received, wraps."),
            CSRField("rx_completions", size=8, description="Completions received, wraps."),
        ])
        self._tx_request_header0    = CSRStatus(32, description="Last Memory Request sent: DWORD 0.")
        self._tx_request_header1    = CSRStatus(32, description="Last Memory Request sent: DWORD 1.")
        self._tx_request_header2    = CSRStatus(32, description="Last Memory Request sent: DWORD 2.")
        self._rx_completion_header0 = CSRStatus(32, description="Last Completion received: DWORD 0.")
        self._rx_completion_header1 = CSRStatus(32, description="Last Completion received: DWORD 1.")
        self._rx_completion_header2 = CSRStatus(32, description="Last Completion received: DWORD 2.")
        self._rx_dwords = CSRStatus(fields=[
            CSRField("last_tlp",  size=16, description="DWORDs of the last received TLP (as presented by the IP)."),
            CSRField("dropped",   size=16, description="DWORDs dropped past TLP header lengths, wraps."),
        ])

        # # #

        def monitor(ep, requests, completions, capture_completions, headers):
            index    = Signal(2) # Index of the next DWORD in the TLP (saturates at 3).
            capture  = Signal()  # Current TLP is captured.
            tlp_type = ep.dat[0:5]
            is_request    = Signal()
            is_completion = Signal()
            self.comb += [
                is_request.eq(tlp_type == 0b00000),
                is_completion.eq(tlp_type == 0b01010),
            ]
            capture_first = is_completion if capture_completions else is_request
            self.sync.pcie += If(ep.valid & ep.ready,
                If(ep.first,
                    index.eq(1),
                    capture.eq(capture_first),
                    If(is_request,    requests.eq(requests + 1)),
                    If(is_completion, completions.eq(completions + 1)),
                    If(capture_first, headers[0].eq(ep.dat)),
                ).Elif(index != 3,
                    index.eq(index + 1),
                    If(capture & (index == 1), headers[1].eq(ep.dat)),
                    If(capture & (index == 2), headers[2].eq(ep.dat)),
                )
            )

        # RX DWORDs per TLP / dropped DWORDs.
        rx_tlp_dwords  = Signal(16)
        rx_last_dwords = Signal(16)
        rx_dropped_cnt = Signal(16)
        self.sync.pcie += [
            If(rx.valid & rx.ready,
                If(rx.first,
                    rx_tlp_dwords.eq(1),
                    If(rx.last, rx_last_dwords.eq(1)),
                ).Else(
                    rx_tlp_dwords.eq(rx_tlp_dwords + 1),
                    If(rx.last, rx_last_dwords.eq(rx_tlp_dwords + 1)),
                )
            ),
            If(rx_dropped, rx_dropped_cnt.eq(rx_dropped_cnt + 1)),
        ]
        self.comb += [
            self._rx_dwords.fields.last_tlp.eq(rx_last_dwords),
            self._rx_dwords.fields.dropped.eq(rx_dropped_cnt),
        ]

        tx_requests    = Signal(8)
        tx_completions = Signal(8)
        rx_requests    = Signal(8)
        rx_completions = Signal(8)
        tx_headers     = [Signal(32) for _ in range(3)]
        rx_headers     = [Signal(32) for _ in range(3)]
        monitor(tx, tx_requests, tx_completions, capture_completions=False, headers=tx_headers)
        monitor(rx, rx_requests, rx_completions, capture_completions=True,  headers=rx_headers)

        self.comb += [
            self._counts.fields.tx_requests.eq(tx_requests),
            self._counts.fields.tx_completions.eq(tx_completions),
            self._counts.fields.rx_requests.eq(rx_requests),
            self._counts.fields.rx_completions.eq(rx_completions),
            self._tx_request_header0.status.eq(tx_headers[0]),
            self._tx_request_header1.status.eq(tx_headers[1]),
            self._tx_request_header2.status.eq(tx_headers[2]),
            self._rx_completion_header0.status.eq(rx_headers[0]),
            self._rx_completion_header1.status.eq(rx_headers[1]),
            self._rx_completion_header2.status.eq(rx_headers[2]),
        ]

# LatticeTLPPHY ------------------------------------------------------------------------------------

class LatticeTLPPHY(LiteXModule):
    endianness    = "big"
    qword_aligned = False

    def __init__(self, platform, pads, data_width=64, pcie_data_width=32, cd="pcie", bar0_size=65536,
        max_payload_size_supported = 256, # Device Capabilities MPS of the IP.
        max_tlp_payload_size       = 128, # Limit for TLPs sent/requested by the core (see below).
        with_tlp_monitor           = True, # Bring-up/debug TLP counters/headers CSRs.
    ):
        # Streams ----------------------------------------------------------------------------------
        self.sink   = stream.Endpoint(phy_layout(data_width))
        self.source = stream.Endpoint(phy_layout(data_width))

        # Registers --------------------------------------------------------------------------------
        self._link_status = CSRStatus(fields=[
            CSRField("status", size=1, values=[
                ("``0b0``", "Link Down."),
                ("``0b1``", "Link Up."),
            ]),
            CSRField("speed", size=4, description="Current Link Speed (``1``: 2.5GT/s, ``2``: 5GT/s, ``3``: 8GT/s)."),
            CSRField("width", size=6, description="Negotiated Link Width (in lanes)."),
        ])
        self._config_status = CSRStatus(fields=[
            CSRField("bus_master_enable",     size=1,  description="Command Bus Master Enable."),
            CSRField("msi_enable",            size=1,  description="MSI Enable."),
            CSRField("msi_mme",               size=3,  description="MSI Multiple Message Enable (log2 of enabled vectors)."),
            CSRField("max_payload_size",      size=3,  description="Device Control Max Payload Size (encoded)."),
            CSRField("max_read_request_size", size=3,  description="Device Control Max Read Request Size (encoded)."),
            CSRField("id",                    size=16, description="Bus/Device/Function (Requester/Completer ID)."),
        ])

        # Parameters/Locals ------------------------------------------------------------------------
        self.platform        = platform
        self.data_width      = data_width
        self.pcie_data_width = pcie_data_width
        self.bar0_size       = bar0_size
        self.bar0_mask       = get_bar_mask(bar0_size)

        # Configuration-Space status (in cd domain).
        self.id                = Signal(16, reset_less=True)
        self.bus_master_enable = Signal()
        self.msi_enable        = Signal()
        self.msi_mme           = Signal(3)
        self.msi_address       = Signal(64)
        self.msi_data          = Signal(16)
        self.msi_mask          = Signal(32)

        self.max_request_size = Signal(16)
        self.max_payload_size = Signal(16)

        # # #

        # Only the single-lane IP (32-bit TLP interface) is supported: with a 128-bit interface,
        # TLPs don't end on word boundaries and the IP doesn't provide which bytes are valid.
        assert data_width in [64, 128]
        assert pcie_data_width == 32, "pcie_data_width must be 32 (single-lane Lattice PCIe IP)."

        # Clocking / Reset -------------------------------------------------------------------------
        self.cd_pcie = ClockDomain()
        self.comb += self.cd_pcie.clk.eq(pads.clk)
        self.specials += AsyncResetSynchronizer(self.cd_pcie, ~pads.rst_n)

        # Configuration-Space Status ---------------------------------------------------------------
        max_payload_size      = Signal(3)
        max_read_request_size = Signal(3)
        link_speed            = Signal(4)
        link_width            = Signal(6)
        for i, o in [
            (pads.id,                    self.id),
            (pads.bus_master_enable,     self.bus_master_enable),
            (pads.msi_enable,            self.msi_enable),
            (pads.msi_mme,               self.msi_mme),
            (pads.msi_address,           self.msi_address),
            (pads.msi_data,              self.msi_data),
            (pads.msi_mask,              self.msi_mask),
            (pads.max_payload_size,      max_payload_size),
            (pads.max_read_request_size, max_read_request_size),
            (pads.link_speed,            link_speed),
            (pads.link_width,            link_width),
        ]:
            if cd == "pcie":
                self.comb += o.eq(i)
            else:
                self.specials += MultiReg(i, o, odomain=cd)

        # Device Control encoding: 128 bytes << code. Clamp to what the IP (payload) and LitePCIe's
        # buffering (litepcie.tlp.common.max_payload_size/max_request_size) support, and to
        # max_tlp_payload_size: the max_request_size clamp also limits the Host's Completions. With
        # this Lattice IP, only 128-byte TLPs were seen to work in an earlier design even though
        # 256 bytes was negotiated, so 128 by default.
        def decode_size(code, size, limit):
            return Case(code, {
                **{n: size.eq(min(128 << n, limit)) for n in range(6)},
                "default": size.eq(128),
            })
        self.comb += [
            decode_size(max_payload_size,      self.max_payload_size, min(max_payload_size_supported, max_tlp_payload_size, max_payload_size_limit)),
            decode_size(max_read_request_size, self.max_request_size, min(max_tlp_payload_size, max_request_size_limit)),
        ]

        self.comb += [
            self._config_status.fields.bus_master_enable.eq(self.bus_master_enable),
            self._config_status.fields.msi_enable.eq(self.msi_enable),
            self._config_status.fields.msi_mme.eq(self.msi_mme),
            self._config_status.fields.max_payload_size.eq(max_payload_size),
            self._config_status.fields.max_read_request_size.eq(max_read_request_size),
            self._config_status.fields.id.eq(self.id),
        ]

        # Link Status ------------------------------------------------------------------------------
        self.comb += [
            self._link_status.fields.status.eq(pads.pl_link_up & pads.dl_link_up & pads.tl_link_up),
            self._link_status.fields.speed.eq(link_speed),
            self._link_status.fields.width.eq(link_width),
        ]

        # TX (FPGA --> HOST) CDC / Data Width Conversion -------------------------------------------
        self.tx_datapath = PHYTXDatapath(
            core_data_width = data_width,
            pcie_data_width = pcie_data_width,
            clock_domain    = cd,
        )
        self.comb += self.sink.connect(self.tx_datapath.sink, omit={"dat", "be"})
        self.comb += dword_endianness_swap(
            src        = self.sink.dat,
            dst        = self.tx_datapath.sink.dat,
            data_width = data_width,
            endianness = "big",
            mode       = "dat",
        )
        self.comb += dword_endianness_swap(
            src        = self.sink.be,
            dst        = self.tx_datapath.sink.be,
            data_width = data_width,
            endianness = "big",
            mode       = "be",
        )
        # vc_tx_* has no byte enables: drop the padding DWORDs of odd-length TLPs.
        self.tx_padding = tx_padding = ClockDomainsRenamer("pcie")(PHYTXPaddingRemover(pcie_data_width))
        self.comb += self.tx_datapath.source.connect(tx_padding.sink)
        s_axis_tx = tx_padding.source

        tx_data_p = Signal(pcie_data_width//8)
        for i in range(pcie_data_width//8):
            self.comb += tx_data_p[i].eq(Reduce("XOR", s_axis_tx.dat[i*8:(i+1)*8]))

        # RX (HOST --> FPGA) CDC / Data Width Conversion -------------------------------------------
        self.rx_datapath = PHYRXDatapath(
            core_data_width = data_width,
            pcie_data_width = pcie_data_width,
            clock_domain    = cd,
        )
        # vc_rx_* has no byte enables: pad odd-length TLPs with be == 0 DWORDs before up-conversion.
        self.rx_padding = rx_padding = ClockDomainsRenamer("pcie")(PHYRXPaddingInserter(pcie_data_width, ratio=data_width//pcie_data_width))
        self.comb += rx_padding.source.connect(self.rx_datapath.sink)
        # Trim TLPs to their header length (safeguard: drops a trash DWORD the IP guides describe; not
        # seen with the single-lane LFD2NX IP, see tlp_monitor rx_dwords).
        self.rx_trimmer = rx_trimmer = ClockDomainsRenamer("pcie")(PHYRXTLPTrimmer(pcie_data_width))
        self.comb += rx_trimmer.source.connect(rx_padding.sink)
        m_axis_rx = rx_trimmer.sink
        self.comb += self.rx_datapath.source.connect(self.source, omit={"dat", "be"})
        self.comb += dword_endianness_swap(
            src        = self.rx_datapath.source.dat,
            dst        = self.source.dat,
            data_width = data_width,
            endianness = "big",
            mode       = "dat",
        )
        self.comb += dword_endianness_swap(
            src        = self.rx_datapath.source.be, # FIXME: Should be adapted.
            dst        = self.source.be,
            data_width = data_width,
            endianness = "big",
            mode       = "be",
        )

        # TLP Receive Interface (HOST --> FPGA) ----------------------------------------------------
        self.comb += [
            m_axis_rx.valid.eq(pads.rx_valid),
            m_axis_rx.first.eq(pads.rx_sop),
            m_axis_rx.last.eq(pads.rx_eop),
            m_axis_rx.dat.eq(pads.rx_data),
            m_axis_rx.be.eq(2**len(m_axis_rx.be) - 1),
            pads.rx_ready.eq(m_axis_rx.ready),

            # Infinite Non-Posted Header credits: flow control is done with rx_ready.
            pads.rx_credit_init.eq(1),
            pads.rx_credit_nh.eq(0),
            pads.rx_credit_nh_inf.eq(1),
            pads.rx_credit_return.eq(1),
        ]

        # TLP Transmit Interface (FPGA --> HOST) ---------------------------------------------------
        self.comb += [
            pads.tx_valid.eq(s_axis_tx.valid),
            pads.tx_sop.eq(s_axis_tx.first),
            pads.tx_eop.eq(s_axis_tx.last),
            pads.tx_eop_n.eq(0),
            pads.tx_data.eq(s_axis_tx.dat),
            pads.tx_datap.eq(tx_data_p),
            s_axis_tx.ready.eq(pads.tx_ready),
        ]

        # TLP Monitor ------------------------------------------------------------------------------
        if with_tlp_monitor:
            self.tlp_monitor = LatticeTLPMonitor(tx=s_axis_tx, rx=m_axis_rx, rx_dropped=rx_trimmer.dropped)
