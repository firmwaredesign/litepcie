#!/usr/bin/env python3

#
# This file is part of LitePCIe.
#
# Copyright (c) 2019-2023 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2020 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

"""
LitePCIe standalone core generator

LitePCIe aims to be directly used as a python package when the SoC is created using LiteX. However,
for some use cases it could be interesting to generate a standalone verilog file of the core:
- integration of the core in a SoC using a more traditional flow.
- need to version/package the core.
- avoid Migen/LiteX dependencies.
- etc...

The standalone core is generated from a YAML configuration file that allows the user to generate
easily a custom configuration of the core.

Current version of the generator is limited to:
- Xilinx 7-Series.
- Xilinx Ultrascale.
- Altera Cyclone V.
- Gowin GW5AT.
- Lattice LFCPNX.
- Lattice LFD2NX.
- Lattice PCIe IP external to the core, connected at the TLP level (LATTICETLPPHY).
"""

import yaml
import argparse
import subprocess

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.cores.clock          import *
from litex.soc.interconnect.csr     import *
from litex.soc.interconnect         import wishbone
from litex.soc.interconnect.axi     import *
from litex.soc.integration.soc      import SoCRegion
from litex.soc.integration.soc import *
from litex.soc.integration.builder  import *

from litepcie.phy.c5pciephy     import C5PCIEPHY
from litepcie.phy.lfcpnxpciephy import LFCPNXPCIEPHY
from litepcie.phy.lfd2nxpciephy import LFD2NXPCIEPHY
from litepcie.phy.latticetlpphy import LatticeTLPPHY
from litepcie.phy.gw5apciephy   import GW5APCIEPHY
from litepcie.phy.s7pciephy     import S7PCIEPHY
from litepcie.phy.uspciephy     import USPCIEPHY
from litepcie.phy.usppciephy    import USPPCIEPHY

from litepcie.core import LitePCIeEndpoint, LitePCIeMSI, LitePCIeMSIMultiVector, LitePCIeMSIX
from litepcie.core.msi import LitePCIeMSITLP

from litepcie.frontend.dma      import LitePCIeDMA
from litepcie.frontend.wishbone import LitePCIeWishboneMaster, LitePCIeWishboneSlave
from litepcie.frontend.axi      import LitePCIeAXISlave
from litepcie.frontend.ptm      import PCIePTMSniffer
from litepcie.frontend.ptm      import PTMCapabilities, PTMRequester

from litepcie.software import generate_litepcie_software_headers

from litex.build.generic_platform import *


# IOs/Interfaces -----------------------------------------------------------------------------------

def get_clk_ios():
    return [
        # Clk / Rst.
        ("clk", 0, Pins(1)),
        ("rst", 0, Pins(1))
    ]

def get_pcie_ios(phy_lanes=4):
    return [
        ("pcie", 0,
            Subsignal("rst_n", Pins(1)),
            Subsignal("clk_p", Pins(1)),
            Subsignal("clk_n", Pins(1)),
            Subsignal("rx_p",  Pins(phy_lanes)),
            Subsignal("rx_n",  Pins(phy_lanes)),
            Subsignal("tx_p",  Pins(phy_lanes)),
            Subsignal("tx_n",  Pins(phy_lanes)),
        ),
    ]

def get_pcie_ios_lattice_lfcpnx(phy_lanes=4):
    # LFCPNXPCIEPHY drives its hard IP directly from pads and needs a few extra
    # signals on top of the generic PCIe IOs: perst (active-low PERST# from the
    # connector), refret/rext (SerDes reference-resistor bias pins). refret/rext
    # are always 4-bit: they're tied to the 4 physical lanes of the SerDes quad
    # itself, regardless of how many are actually used for the PCIe link.
    return [
        ("pcie", 0,
            Subsignal("rst_n",  Pins(1)),
            Subsignal("clk_p",  Pins(1)),
            Subsignal("clk_n",  Pins(1)),
            Subsignal("rx_p",   Pins(phy_lanes)),
            Subsignal("rx_n",   Pins(phy_lanes)),
            Subsignal("tx_p",   Pins(phy_lanes)),
            Subsignal("tx_n",   Pins(phy_lanes)),
            Subsignal("perst",  Pins(1)),
            Subsignal("refret", Pins(4)),
            Subsignal("rext",   Pins(4)),
        ),
    ]

def get_clkin125_ios():
    # LFCPNXPCIEPHY's LMMI/config logic needs a separate 125MHz reference Clk,
    # provided by the User (independent of the PCIe refclk).
    return [("clkin125", 0, Pins(1))]

def get_pcie_ios_lattice_lfd2nx():
    # LFD2NXPCIEPHY's "pci" IP is only ever generated single-lane, so unlike
    # get_pcie_ios_lattice_lfcpnx() every signal here (including refret/rext) is
    # fixed scalar width -- there's no phy_lanes to parametrize on.
    return [
        ("pcie", 0,
            Subsignal("rst_n",  Pins(1)),
            Subsignal("clk_p",  Pins(1)),
            Subsignal("clk_n",  Pins(1)),
            Subsignal("rx_p",   Pins(1)),
            Subsignal("rx_n",   Pins(1)),
            Subsignal("tx_p",   Pins(1)),
            Subsignal("tx_n",   Pins(1)),
            Subsignal("perst",  Pins(1)),
            Subsignal("refret", Pins(1)),
            Subsignal("rext",   Pins(1)),
        ),
    ]

def get_clkin_usr_ios():
    # LFD2NXPCIEPHY needs a separate User-provided Clk on clk_usr_i (independent
    # of the PCIe refclk); minimum frequency depends on the target link speed
    # baked into the generated archive -- see litepcie/phy/lfd2nxpciephy.py.
    return [("clkin_usr", 0, Pins(1))]

def get_pcie_ios_lattice_tlp(pcie_data_width=32):
    # LatticeTLPPHY: the Lattice PCIe IP is instantiated in the User top level, next to the core.
    # The core connects to the IP's TLP interface (vc_rx_*/vc_tx_*) and gets link/configuration-
    # space status from User logic (ucfg reader) -- see litepcie/phy/latticetlpphy.py.
    return [
        ("pcie", 0,
            # Clk/Rst / Link Status (clk: clock fed to the IP's clk_usr_i; u_*_link_up_o).
            Subsignal("clk",                   Pins(1)),
            Subsignal("rst_n",                 Pins(1)),
            Subsignal("pl_link_up",            Pins(1)),
            Subsignal("dl_link_up",            Pins(1)),
            Subsignal("tl_link_up",            Pins(1)),

            # TLP Receive Interface (IP vc_rx_*).
            Subsignal("rx_valid",              Pins(1)),
            Subsignal("rx_ready",              Pins(1)),
            Subsignal("rx_sop",                Pins(1)),
            Subsignal("rx_eop",                Pins(1)),
            Subsignal("rx_data",               Pins(pcie_data_width)),
            Subsignal("rx_credit_init",        Pins(1)),
            Subsignal("rx_credit_nh",          Pins(12)),
            Subsignal("rx_credit_nh_inf",      Pins(1)),
            Subsignal("rx_credit_return",      Pins(1)),

            # TLP Transmit Interface (IP vc_tx_*).
            Subsignal("tx_valid",              Pins(1)),
            Subsignal("tx_ready",              Pins(1)),
            Subsignal("tx_sop",                Pins(1)),
            Subsignal("tx_eop",                Pins(1)),
            Subsignal("tx_eop_n",              Pins(1)),
            Subsignal("tx_data",               Pins(pcie_data_width)),
            Subsignal("tx_datap",              Pins(pcie_data_width//8)),

            # Configuration-Space Status (from ucfg reader).
            Subsignal("id",                    Pins(16)),
            Subsignal("bus_master_enable",     Pins(1)),
            Subsignal("max_payload_size",      Pins(3)),
            Subsignal("max_read_request_size", Pins(3)),
            Subsignal("link_speed",            Pins(4)),
            Subsignal("link_width",            Pins(6)),
            Subsignal("msi_enable",            Pins(1)),
            Subsignal("msi_mme",               Pins(3)),
            Subsignal("msi_address",           Pins(64)),
            Subsignal("msi_data",              Pins(16)),
            Subsignal("msi_mask",              Pins(32)),
        ),
    ]

def get_axi_dma_ios(_id, data_width, with_writer=True, with_reader=True):
    ios = []

    # Enables.
    enables = []
    if with_writer:
        enables += [Subsignal("writer_enable", Pins(1))]
    if with_reader:
        enables += [Subsignal("reader_enable", Pins(1))]
    ios += [("dma{}_status".format(_id), 0, *enables)]

    # DMA Writer AXI.
    if with_writer:
        ios += [
            ("dma{}_writer_axi".format(_id), 0,
                Subsignal("tvalid", Pins(1)),
                Subsignal("tready", Pins(1)),
                Subsignal("tlast",  Pins(1)),
                Subsignal("tdata",  Pins(data_width)),
                Subsignal("tuser",  Pins(1)), # Use for tfirst.
            )
        ]

    # DMA Reader AXI.
    if with_reader:
        ios += [
            ("dma{}_reader_axi".format(_id), 0,
                Subsignal("tvalid", Pins(1)),
                Subsignal("tready", Pins(1)),
                Subsignal("tlast",  Pins(1)),
                Subsignal("tdata",  Pins(data_width)),
                Subsignal("tuser",  Pins(1)), # Use for tfirst.
            )
        ]

    return ios

def get_msi_irqs_ios(width=16):
    return [("msi_irqs", 0, Pins(width))]


def get_eth_ios(dw=8):
    # PHY-side stream of the Ethernet MAC, handed to the User's module: eth_tx_* is driven by the
    # core (Host --> User), eth_rx_* by the User (User --> Host). Ethernet frames are carried without
    # preamble/FCS unless ethernet_preamble_crc is set.
    def stream_ios(name):
        return (name, 0,
            Subsignal("valid",   Pins(1)),
            Subsignal("ready",   Pins(1)),
            Subsignal("first",   Pins(1)),
            Subsignal("last",    Pins(1)),
            Subsignal("data",    Pins(dw)),
            Subsignal("last_be", Pins(dw//8)),
            Subsignal("error",   Pins(dw//8)),
        )
    return [stream_ios("eth_tx"), stream_ios("eth_rx")]

# Ethernet User PHY --------------------------------------------------------------------------------

class UserEthPHY(LiteXModule):
    """Ethernet "PHY" handing the MAC's PHY-side stream to the User instead of driving a real PHY.

    The User connects a custom module to the eth_tx_*/eth_rx_* IOs and exchanges Ethernet frames with
    the Host, which sees a standard LiteEth MAC (handled by the LitePCIe driver's network interface).
    With with_loopback, the stream is looped back inside the core instead (bring-up/test).
    """
    def __init__(self, platform, dw=8, with_preamble_crc=False, with_padding=False, with_loopback=False):
        from liteeth.common import eth_phy_description
        self.dw                = dw
        self.with_preamble_crc = with_preamble_crc
        self.with_padding      = with_padding
        self.sink   = sink   = stream.Endpoint(eth_phy_description(dw)) # MAC  --> User (TX).
        self.source = source = stream.Endpoint(eth_phy_description(dw)) # User --> MAC  (RX).

        # # #

        # The MAC's PHY-side datapath runs in the eth_tx/eth_rx clock domains: use the core's Clk.
        self.cd_eth_tx = ClockDomain()
        self.cd_eth_rx = ClockDomain()
        self.comb += [
            self.cd_eth_tx.clk.eq(ClockSignal("sys")),
            self.cd_eth_tx.rst.eq(ResetSignal("sys")),
            self.cd_eth_rx.clk.eq(ClockSignal("sys")),
            self.cd_eth_rx.rst.eq(ResetSignal("sys")),
        ]

        # Loopback (TX --> RX) or User IOs.
        if with_loopback:
            self.comb += sink.connect(source)
        else:
            platform.add_extension(get_eth_ios(dw))
            tx_pads = platform.request("eth_tx")
            rx_pads = platform.request("eth_rx")
            self.comb += [
                # MAC --> User.
                tx_pads.valid.eq(sink.valid),
                sink.ready.eq(tx_pads.ready),
                tx_pads.first.eq(sink.first),
                tx_pads.last.eq(sink.last),
                tx_pads.data.eq(sink.data),
                tx_pads.last_be.eq(sink.last_be),
                tx_pads.error.eq(sink.error),

                # User --> MAC.
                source.valid.eq(rx_pads.valid),
                rx_pads.ready.eq(source.ready),
                source.first.eq(rx_pads.first),
                source.last.eq(rx_pads.last),
                source.data.eq(rx_pads.data),
                source.last_be.eq(rx_pads.last_be),
                source.error.eq(rx_pads.error),
            ]

def get_ptm_ios(phy_lanes=4):
    return [
        ("ptm", 0,
            Subsignal("time_clk", Pins(1)),
            Subsignal("time_rst", Pins(1)),
            Subsignal("time_ns",  Pins(64)),
        ),
    ]

# CRG ----------------------------------------------------------------------------------------------

class LitePCIeCRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, clk_external):
        self.cd_sys = ClockDomain()

        # # #

        # Create/Get Clk/Rst IOs.
        platform.add_extension(get_clk_ios())
        clk = platform.request("clk")
        rst = platform.request("rst")

        # Get PCIe Clk/Rst.
        pcie_clk = ClockSignal("pcie")
        pcie_rst = ResetSignal("pcie")

        # External Clk mode: Clk is provided by the User logic.
        if clk_external:
            self.comb += self.cd_sys.clk.eq(clk)
            self.specials += AsyncResetSynchronizer(self.cd_sys, rst | pcie_rst)

        # Internal Clk mode: Clk is provided to the User logic by the LitePCIe standalone core.
        else:
            self.comb += [
                clk.eq(pcie_clk),
                rst.eq(pcie_rst),
                self.cd_sys.clk.eq(pcie_clk),
                self.cd_sys.rst.eq(pcie_rst),
            ]

# Core ---------------------------------------------------------------------------------------------

class LitePCIeCore(SoCMini):
    SoCMini.mem_map["csr"] = 0x00000000
    SoCMini.csr_map = {
        "ctrl"             : 0,
        "crg"              : 1,
        "pcie_phy"         : 2,
        "pcie_msi"         : 3,
        "pcie_msi_table"   : 4,
        "ptm_capabilities" : 5,
        "ptm_requester"    : 6,
    }
    def __init__(self, platform, core_config):
        # LFCPNXPCIEPHY/LFD2NXPCIEPHY need extra PCIe pads (perst/refret/rext) and a
        # separate User Clk that the generic PCIe IOs don't provide.
        if core_config["phy"] is LFCPNXPCIEPHY:
            platform.add_extension(get_pcie_ios_lattice_lfcpnx(core_config["phy_lanes"]))
            platform.add_extension(get_clkin125_ios())
        elif core_config["phy"] is LFD2NXPCIEPHY:
            platform.add_extension(get_pcie_ios_lattice_lfd2nx())
            platform.add_extension(get_clkin_usr_ios())
        elif core_config["phy"] is LatticeTLPPHY:
            platform.add_extension(get_pcie_ios_lattice_tlp(core_config["phy_pcie_data_width"]))
        else:
            platform.add_extension(get_pcie_ios(core_config["phy_lanes"]))
        platform.add_extension(get_msi_irqs_ios(width=core_config["msi_irqs"]))
        sys_clk_freq = float(core_config.get("clk_freq", 125e6))

        # Parameters -------------------------------------------------------------------------------

        ep_address_width        = core_config.get("ep_address_width", 32)
        ep_max_pending_requests = core_config.get("ep_max_pending_requests", 4)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq,
            csr_data_width = 32,
            csr_ordering   = core_config.get("csr_ordering", "big"),
            ident          = "LitePCIe standalone core",
            ident_version  = True
        )

        # CRG --------------------------------------------------------------------------------------
        clk_external = core_config.get("clk_external", False)
        self.crg = LitePCIeCRG(platform, sys_clk_freq, clk_external)

        # Control ----------------------------------------------------------------------------------
        if core_config.get("ctrl", False):
            axi = AXILiteInterface(data_width=32, address_width=32)
            platform.add_extension(axi.get_ios("ctrl_axi_lite"))
            axi_pads = platform.request("ctrl_axi_lite")
            self.comb += axi.connect_to_pads(axi_pads, mode="slave")
            self.bus.add_master(name="ctrl", master=axi)

        # PCIe PHY ---------------------------------------------------------------------------------
        self.pcie_phy = core_config["phy"](platform, platform.request("pcie"),
            pcie_data_width = core_config.get("phy_pcie_data_width", 64),
            data_width      = core_config["phy_data_width"],
            bar0_size       = core_config["phy_bar0_size"])

        # PCIe Endpoint ----------------------------------------------------------------------------
        self.pcie_endpoint = LitePCIeEndpoint(self.pcie_phy,
            endianness           = self.pcie_phy.endianness,
            address_width        = ep_address_width,
            max_pending_requests = ep_max_pending_requests,
            with_ptm             = core_config.get("ptm", False),
        )

        # PCIe Wishbone Master ---------------------------------------------------------------------
        pcie_wishbone_master = LitePCIeWishboneMaster(self.pcie_endpoint,
            qword_aligned = self.pcie_phy.qword_aligned)
        self.submodules += pcie_wishbone_master
        self.bus.add_master(master=pcie_wishbone_master.wishbone)

        # PCIe MMAP Master -------------------------------------------------------------------------
        if core_config.get("mmap", False):
            mmap_base        = core_config["mmap_base"]
            mmap_size        = core_config["mmap_size"]
            mmap_translation = core_config.get("mmap_translation", 0x00000000)
            mmap_region      = SoCRegion(origin=mmap_base, size=mmap_size, cached=False)
            wb = wishbone.Interface(data_width=32)
            self.bus.add_slave(name="mmap", slave=wb, region=mmap_region)
            axi = AXILiteInterface(data_width=32, address_width=32)
            wb2axi = Wishbone2AXILite(wb, axi, base_address=-mmap_translation)
            self.submodules += wb2axi
            platform.add_extension(axi.get_ios("mmap_axi_lite"))
            axi_pads = platform.request("mmap_axi_lite")
            self.comb += axi.connect_to_pads(axi_pads, mode="master")

        # PCIe MMAP Slave --------------------------------------------------------------------------
        if core_config.get("mmap_slave", False):
            # AXI-Full
            if core_config.get("mmap_slave_axi_full", False):
                pcie_axi_slave = LitePCIeAXISlave(self.pcie_endpoint, data_width=128)
                self.submodules += pcie_axi_slave
                platform.add_extension(pcie_axi_slave.axi.get_ios("mmap_slave_axi"))
                axi_pads = platform.request("mmap_slave_axi")
                self.comb += pcie_axi_slave.axi.connect_to_pads(axi_pads, mode="slave")
            # AXI-Lite
            else:
                platform.add_extension(axi.get_ios("mmap_slave_axi_lite"))
                axi_pads = platform.request("mmap_slave_axi_lite")
                wb = wishbone.Interface(data_width=32)
                axi = AXILiteInterface(data_width=32, address_width=32)
                self.comb += axi.connect_to_pads(axi_pads, mode="slave")
                axi2wb = AXILite2Wishbone(axi, wb)
                self.submodules += axi2wb
                pcie_wishbone_slave = LitePCIeWishboneSlave(self.pcie_endpoint,
                    qword_aligned=self.pcie_phy.qword_aligned)
                self.submodules += pcie_wishbone_slave
                self.comb += wb.connect(pcie_wishbone_slave.wishbone)

        # PCIe DMA ---------------------------------------------------------------------------------
        pcie_dmas = []

        # Parameters.
        # -----------

        dmas_params = []

        class DMAParams:
            def __init__(self, writer, reader, buffering, loopback, synchronizer, monitor, data_width):
                self.writer       = writer
                self.reader       = reader
                self.buffering    = buffering
                self.loopback     = loopback
                self.synchronizer = synchronizer
                self.monitor      = monitor
                self.data_width   = data_width

        # DMA Channels configured separately.
        if isinstance(core_config.get("dma_channels"), dict):
            print(core_config.get("dma_channels"))
            for name, params in core_config["dma_channels"].items():
                dma_params = DMAParams(
                    data_width   = params.get("dma_data_width",    core_config["phy_data_width"]),
                    writer       = params.get("dma_writer",        True),
                    reader       = params.get("dma_reader",        True),
                    buffering    = params.get("dma_buffering",     1024),
                    loopback     = params.get("dma_loopback",      True),
                    synchronizer = params.get("dma_synchronizer", False),
                    monitor      = params.get("dma_monitor",      False),
                )
                dmas_params.append(dma_params)

        # DMA Channels configured identically.
        else:
            for n in range(core_config["dma_channels"]):
                dma_params = DMAParams(
                    data_width   = core_config.get("dma_data_width",    core_config["phy_data_width"]),
                    writer       = core_config.get("dma_writer",        True),
                    reader       = core_config.get("dma_reader",        True),
                    buffering    = core_config.get("dma_buffering",     1024),
                    loopback     = core_config.get("dma_loopback",      True),
                    synchronizer = core_config.get("dma_synchronizer", False),
                    monitor      = core_config.get("dma_monitor",      False),
                )
                dmas_params.append(dma_params)

        self.add_constant("DMA_CHANNELS",   len(dmas_params))
        self.add_constant("DMA_ADDR_WIDTH", ep_address_width)

        # PCIe DMAs.
        # ----------
        for i, dma_params in enumerate(dmas_params):
            # DMA.
            # ----
            pcie_dma = LitePCIeDMA(self.pcie_phy, self.pcie_endpoint,
                address_width     = ep_address_width,
                data_width        = dma_params.data_width,
                with_writer       = dma_params.writer,
                with_reader       = dma_params.reader,
                with_buffering    = dma_params.buffering != 0,
                buffering_depth   = dma_params.buffering,
                with_loopback     = dma_params.loopback,
                with_synchronizer = dma_params.synchronizer,
                with_monitor      = dma_params.monitor,
            )
            # DMA Endpoint Buffers (For timings).
            # -------------------------------
            pcie_dma = stream.BufferizeEndpoints({"sink"   : stream.DIR_SINK})(pcie_dma)
            pcie_dma = stream.BufferizeEndpoints({"source" : stream.DIR_SOURCE})(pcie_dma)
            self.add_module(f"pcie_dma{i}", pcie_dma)

            # DMA IOs.
            # --------
            platform.add_extension(get_axi_dma_ios(i,
                data_width  = dma_params.data_width,
                with_writer = dma_params.writer,
                with_reader = dma_params.reader,
            ))
            dma_status_ios = platform.request(f"dma{i}_status")

            # DMA Writer <-> IOs Connection.
            # ------------------------------
            if dma_params.writer:
                dma_writer_ios = platform.request(f"dma{i}_writer_axi")
                self.comb += [
                    # Status IOs.
                    dma_status_ios.writer_enable.eq(pcie_dma.writer.enable),

                    # Writer IOs.
                    pcie_dma.sink.valid.eq(dma_writer_ios.tvalid & pcie_dma.writer.enable),
                    dma_writer_ios.tready.eq(pcie_dma.sink.ready & pcie_dma.writer.enable),
                    pcie_dma.sink.last.eq(dma_writer_ios.tlast),
                    pcie_dma.sink.data.eq(dma_writer_ios.tdata),
                    pcie_dma.sink.first.eq(dma_writer_ios.tuser),
                ]

            # DMA Reader <-> IOs Connection.
            # ------------------------------
            if dma_params.reader:
                dma_reader_ios = platform.request(f"dma{i}_reader_axi")
                self.comb += [
                    # Status IOs.
                    dma_status_ios.reader_enable.eq(pcie_dma.reader.enable),

                    # Reader IOs.
                    dma_reader_ios.tvalid.eq(pcie_dma.source.valid & pcie_dma.reader.enable),
                    pcie_dma.source.ready.eq(dma_reader_ios.tready | ~pcie_dma.reader.enable),
                    dma_reader_ios.tlast.eq(pcie_dma.source.last),
                    dma_reader_ios.tdata.eq(pcie_dma.source.data),
                    dma_reader_ios.tuser.eq(pcie_dma.source.first),
                ]

        # Ethernet ---------------------------------------------------------------------------------
        # LiteEth MAC with its slots mapped in BAR0: the Host sees a standard LiteEth MAC (handled by
        # the LitePCIe driver's network interface) and the User exchanges Ethernet frames on the
        # PHY-side stream (see UserEthPHY).
        if core_config.get("ethernet", False):
            from liteeth.mac import LiteEthMAC
            eth_data_width = core_config.get("ethernet_data_width", 32)
            assert eth_data_width == 32, "ethernet_data_width must be 32 (BAR0/Wishbone data width)."
            eth_nrxslots = core_config.get("ethernet_rx_slots", 2)
            eth_ntxslots = core_config.get("ethernet_tx_slots", 2)

            self.eth_phy = eth_phy = UserEthPHY(platform,
                dw                = core_config.get("ethernet_phy_data_width", 8),
                with_preamble_crc = core_config.get("ethernet_preamble_crc",   False),
                with_padding      = core_config.get("ethernet_padding",        False),
                with_loopback     = core_config.get("ethernet_loopback",       False),
            )
            self.ethmac = ethmac = LiteEthMAC(
                phy        = eth_phy,
                dw         = eth_data_width,
                interface  = "wishbone",
                # The Host is little-endian and the driver moves frames in/out of the slots with
                # memcpy_to/fromio (byte order = memory order), so the MAC must not reorder the bytes
                # within a 32-bit word. With "big" the reversal cancels out in loopback for frames
                # whose length is a multiple of 4, but the last partial word is emitted from the
                # wrong byte lanes (a 1514-byte frame comes back with its last 2 bytes replaced by
                # padding).
                endianness = core_config.get("ethernet_endianness", "little"),
                # The Host only reads the RX slots and only writes the TX slots (and so does the
                # driver), so each slot memory keeps a single writer. Making the RX slots writable
                # from the Host adds a second, byte-granular write process on the same memory, which
                # Synplify Pro (Radiant) rejects with:
                #   @E: CS152 |Only one always block can assign given variable mac_sram_writer_slot0
                nrxslots   = eth_nrxslots, rxslots_read_only  = True,
                ntxslots   = eth_ntxslots, txslots_write_only = False,
            )

            # Map the MAC's RX/TX slots in BAR0, after the CSR region.
            eth_slot_size = ethmac.slot_size.constant
            eth_rx_size   = eth_nrxslots*eth_slot_size
            eth_tx_size   = eth_ntxslots*eth_slot_size
            eth_origin    = core_config.get("ethernet_base", 0x10000)
            self.bus.add_region("ethmac", SoCRegion(
                origin = eth_origin,
                size   = eth_rx_size + eth_tx_size,
                linker = True,
                cached = False,
            ))
            self.bus.add_slave(name="ethmac_rx", slave=ethmac.bus_rx, region=SoCRegion(
                origin = eth_origin,
                size   = eth_rx_size,
                cached = False,
            ))
            self.bus.add_slave(name="ethmac_tx", slave=ethmac.bus_tx, region=SoCRegion(
                origin = eth_origin + eth_rx_size,
                size   = eth_tx_size,
                cached = False,
            ))

        # PCIe MSI ---------------------------------------------------------------------------------
        if core_config.get("msi_x", False):
            assert core_config["msi_irqs"] <= 32
            msi_x_default_enable = int(core_config.get("msi_x_default_enable", False))
            self.pcie_msi = LitePCIeMSIX(self.pcie_endpoint, width=64, default_enable=msi_x_default_enable)
            self.comb += self.pcie_msi.irqs[32:32+core_config["msi_irqs"]].eq(platform.request("msi_irqs"))
        else:
            assert core_config["msi_irqs"] <= 16
            if core_config.get("msi_multivector", False):
                self.pcie_msi = LitePCIeMSIMultiVector(width=32)
            else:
                self.pcie_msi = LitePCIeMSI(width=32)
            # PHY without MSI request interface: send MSIs as Memory Write TLPs.
            if hasattr(self.pcie_phy, "msi_address"):
                self.pcie_msi_tlp = LitePCIeMSITLP(self.pcie_endpoint,
                    enable  = self.pcie_phy.msi_enable,
                    address = self.pcie_phy.msi_address,
                    data    = self.pcie_phy.msi_data,
                    mme     = self.pcie_phy.msi_mme,
                    mask    = self.pcie_phy.msi_mask,
                )
                self.comb += self.pcie_msi.source.connect(self.pcie_msi_tlp.sink)
            else:
                self.comb += self.pcie_msi.source.connect(self.pcie_phy.msi)
            self.comb += self.pcie_msi.irqs[16:16+core_config["msi_irqs"]].eq(platform.request("msi_irqs"))
        self.interrupts = {}
        if core_config.get("ethernet", False):
            self.interrupts["ethmac"] = self.ethmac.ev.irq
        for i in range(len(dmas_params)):
            pcie_dma = getattr(self, f"pcie_dma{i}")
            if hasattr(pcie_dma, "writer"):
                self.interrupts[f"pcie_dma{i}_writer"] = pcie_dma.writer.irq
            if hasattr(pcie_dma, "reader"):
                self.interrupts[f"pcie_dma{i}_reader"] = pcie_dma.reader.irq
        for i, (k, v) in enumerate(sorted(self.interrupts.items())):
            self.comb += self.pcie_msi.irqs[i].eq(v)
            self.add_constant(k.upper() + "_INTERRUPT", i)
        assert len(self.interrupts.keys()) <= 16

        # PCIe PTM ---------------------------------------------------------------------------------
        if core_config.get("ptm", False):

            # PCIe PTM Sniffer ---------------------------------------------------------------------

            # Since Xilinx PHY does not allow redirecting PTM TLP Messages to the AXI inferface, we have
            # to sniff the GTPE2 -> PCIE2 RX Data to re-generate PTM TLP Messages.

            # Sniffer Signals.
            # ----------------
            sniffer_rst_n   = Signal()
            sniffer_clk     = Signal()
            sniffer_rx_data = Signal(16)
            sniffer_rx_ctl  = Signal(2)

            # Sniffer Tap.
            # ------------
            rx_data = Signal(16)
            rx_ctl  = Signal(2)
            self.sync.pclk += rx_data.eq(rx_data + 1)
            self.sync.pclk += rx_ctl.eq(rx_ctl + 1)
            self.specials += Instance("sniffer_tap",
                i_rst_n_in    = 1,
                i_clk_in     = ClockSignal("pclk"),
                i_rx_data_in = rx_data, # /!\ Fake, will be re-connected post-synthesis /!\.
                i_rx_ctl_in  = rx_ctl,  # /!\ Fake, will be re-connected post-synthesis /!\.

                o_rst_n_out   = sniffer_rst_n,
                o_clk_out     = sniffer_clk,
                o_rx_data_out = sniffer_rx_data,
                o_rx_ctl_out  = sniffer_rx_ctl,
            )

            # Sniffer.
            # --------
            self.pcie_ptm_sniffer = PCIePTMSniffer(
                rx_rst_n = sniffer_rst_n,
                rx_clk   = sniffer_clk,
                rx_data  = sniffer_rx_data,
                rx_ctrl  = sniffer_rx_ctl,
            )
            self.pcie_ptm_sniffer.add_sources(platform)

            # Sniffer Post-Synthesis connections.
            # -----------------------------------
            pcie_ptm_sniffer_connections = []
            for n in range(2):
                pcie_ptm_sniffer_connections.append((
                    f"pcie_s7/inst/inst/gt_top_i/gt_rx_data_k_wire_filter[{n}]", # Src.
                    f"pcie_ptm_sniffer_tap/rx_ctl_in[{n}]",                      # Dst.
                ))
            for n in range(16):
                pcie_ptm_sniffer_connections.append((
                    f"pcie_s7/inst/inst/gt_top_i/gt_rx_data_wire_filter[{n}]", # Src.
                    f"pcie_ptm_sniffer_tap/rx_data_in[{n}]",                   # Dst.
                ))
            for _from, _to in pcie_ptm_sniffer_connections:
                platform.toolchain.pre_optimize_commands.append(f"set pin_driver [get_nets -of [get_pins {_to}]]")
                platform.toolchain.pre_optimize_commands.append(f"disconnect_net -net $pin_driver -objects {_to}")
                platform.toolchain.pre_optimize_commands.append(f"connect_net -hier -net {_from} -objects {_to}")

            # PTM IOs ------------------------------------------------------------------------------
            platform.add_extension(get_ptm_ios())
            ptm_ios = platform.request("ptm")

            # PTM Capabilities ---------------------------------------------------------------------
            self.ptm_capabilities = PTMCapabilities(
                pcie_endpoint     = self.pcie_endpoint,
                requester_capable = True,
            )

            # PTM Requester ------------------------------------------------------------------------
            self.ptm_requester = PTMRequester(
                pcie_endpoint    = self.pcie_endpoint,
                pcie_ptm_sniffer = self.pcie_ptm_sniffer,
                sys_clk_freq     = sys_clk_freq,
            )
            self.comb += [
                self.ptm_requester.time_clk.eq(ptm_ios.time_clk),
                self.ptm_requester.time_rst.eq(ptm_ios.time_rst),
                self.ptm_requester.time.eq(ptm_ios.time_ns)
            ]

    def generate_documentation(self, build_name, **kwargs):
        from litex.soc.doc import generate_docs
        generate_docs(self, "documentation".format(build_name),
            project_name = "LitePCIe standalone core",
            author       = "Enjoy-Digital")
        os.system("sphinx-build -M html documentation/ documentation/_build".format(build_name, build_name))

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LitePCIe standalone core generator")
    parser.add_argument("config",       help="YAML config file")
    parser.add_argument("--doc",        action="store_true", help="Build documentation")
    parser.add_argument("--output-dir", default="build",     help="Build output directory")
    parser.add_argument("--header-dir", default="./",        help="Generated header output directory")
    args = parser.parse_args()
    core_config = yaml.load(open(args.config).read(), Loader=yaml.Loader)

    # Convert YAML elements to Python/LiteX --------------------------------------------------------
    for k, v in core_config.items():
        replaces = {"False": False, "True": True, "None": None}
        for r in replaces.keys():
            if v == r:
                core_config[k] = replaces[r]

    # Generate core --------------------------------------------------------------------------------
    if core_config["phy"]  == "C5PCIEPHY":
        from litex.build.altera import AlteraPlatform
        platform = AlteraPlatform("", io=[])
        core_config["phy"] = C5PCIEPHY
    elif core_config["phy"] == "LFCPNXPCIEPHY":
        from litex.build.lattice import LatticePlatform
        platform = LatticePlatform(core_config["phy_device"], io=[], toolchain="radiant")
        core_config.setdefault("msi_x", True)
        # Generated Lattice hard IP archives only exist for these lane counts (see
        # LFCPNXPCIEPHY.ip_urls); fail early with a clear message instead of the
        # assert inside LFCPNXPCIEPHY. pcie_data_width is fixed by the IP itself
        # (32-bit/lane), so default it from phy_lanes rather than a flat constant.
        phy_lanes = core_config.get("phy_lanes", 4)
        assert phy_lanes in [1, 4], \
            "LFCPNXPCIEPHY only supports phy_lanes: 1 or 4 (Lattice's PCIe hard IP is generated per lane count)."
        core_config.setdefault("phy_pcie_data_width", {1: 32, 4: 128}[phy_lanes])
        core_config["phy"] = LFCPNXPCIEPHY
    elif core_config["phy"] == "LFD2NXPCIEPHY":
        from litex.build.lattice import LatticePlatform
        platform = LatticePlatform(core_config["phy_device"], io=[], toolchain="radiant")
        # This IP is baked with MSI enabled/MSI-X disabled at generation time (see
        # litepcie/phy/lfd2nxpciephy.py); default the LitePCIe-side MSI mechanism to match.
        core_config.setdefault("msi_x", False)
        # Only ever generated single-lane -- no lane count to select an archive by.
        assert core_config.get("phy_lanes", 1) == 1, \
            "LFD2NXPCIEPHY only supports phy_lanes: 1 (this IP is generated single-lane only)."
        core_config.setdefault("phy_pcie_data_width", 32)
        core_config["phy"] = LFD2NXPCIEPHY
    elif core_config["phy"] == "LATTICETLPPHY":
        from litex.build.lattice import LatticePlatform
        platform = LatticePlatform(core_config["phy_device"], io=[], toolchain="radiant")
        # The external IP owns the MSI capability (MSI-X disabled); MSIs are sent as TLPs.
        core_config.setdefault("msi_x", False)
        assert core_config.get("phy_lanes", 1) == 1, \
            "LATTICETLPPHY only supports phy_lanes: 1 (32-bit TLP interface)."
        core_config.setdefault("phy_pcie_data_width", 32)
        core_config["phy"] = LatticeTLPPHY
    elif core_config["phy"] == "GW5APCIEPHY":
        from litex.build.gowin import GowinPlatform
        platform = GowinPlatform(core_config["phy_device"], io=[], toolchain="gowin")
        core_config.setdefault("phy_pcie_data_width", 256)
        core_config["phy"] = GW5APCIEPHY
    elif core_config["phy"] == "S7PCIEPHY":
        from litex.build.xilinx import XilinxPlatform
        platform = XilinxPlatform(core_config["phy_device"], io=[], toolchain="vivado")
        core_config["phy"] = S7PCIEPHY
    elif core_config["phy"] == "USPCIEPHY":
        from litex.build.xilinx import XilinxPlatform
        platform = XilinxPlatform(core_config["phy_device"], io=[], toolchain="vivado")
        core_config["phy"] = USPCIEPHY
    elif core_config["phy"] == "USPPCIEPHY":
        from litex.build.xilinx import XilinxPlatform
        platform = XilinxPlatform(core_config["phy_device"], io=[], toolchain="vivado")
        core_config["phy"] = USPPCIEPHY
    else:
        raise ValueError("Unsupported PCIe PHY: {}".format(core_config["phy"]))
    soc      = LitePCIeCore(platform, core_config)
    builder  = Builder(soc, output_dir=args.output_dir, compile_gateware=False)
    builder.build(build_name="litepcie_core")
    generate_litepcie_software_headers(soc, args.header_dir)

    if args.doc:
        soc.generate_documentation("litepcie_core")

if __name__ == "__main__":
    main()
