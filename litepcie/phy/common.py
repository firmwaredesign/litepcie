#
# This file is part of LitePCIe.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg

from litepcie.common import *

# Helpers ------------------------------------------------------------------------------------------

def get_bar_size_config(size):
    """Return the exact Xilinx PCIe IP scale/size pair for a 32-bit BAR."""
    if (size < 128) or (size > 2*GB) or (size & (size - 1)):
        raise ValueError("32-bit PCIe BAR size must be a power of two between 128 bytes and 2 GiB")

    values = [1, 2, 4, 8, 16, 32, 64, 128, 256, 512]
    for scale, unit in [
        ("Gigabytes", GB),
        ("Megabytes", MB),
        ("Kilobytes", KB),
        ("Bytes",       1),
    ]:
        if (size % unit) == 0 and (size // unit) in values:
            return scale, size // unit

    raise ValueError(f"PCIe BAR size 0x{size:x} cannot be represented by the Xilinx PCIe IP")

# TX Datapath --------------------------------------------------------------------------------------

class PHYTXDatapath(Module):
    def __init__(self, core_data_width, pcie_data_width, clock_domain):
        self.sink   = sink   = stream.Endpoint(phy_layout(core_data_width))
        self.source = source = stream.Endpoint(phy_layout(pcie_data_width))

        # # #

        if (clock_domain == "pcie") and (core_data_width == pcie_data_width):
            self.comb += sink.connect(source)
        else:
            pipe_valid = stream.PipeValid(phy_layout(core_data_width))
            pipe_valid = ClockDomainsRenamer(clock_domain)(pipe_valid)
            cdc        = stream.ClockDomainCrossing(
                layout          = phy_layout(core_data_width),
                cd_from         = clock_domain,
                cd_to           = "pcie",
                with_common_rst = True,
                depth           = 16,
            )
            converter  = stream.StrideConverter(phy_layout(core_data_width), phy_layout(pcie_data_width))
            converter  = ClockDomainsRenamer("pcie")(converter)
            pipe_ready = stream.PipeReady(phy_layout(pcie_data_width))
            pipe_ready = ClockDomainsRenamer("pcie")(pipe_ready)
            self.submodules += pipe_valid, cdc, converter, pipe_ready
            self.comb += [
                sink.connect(pipe_valid.sink),
                pipe_valid.source.connect(cdc.sink),
                cdc.source.connect(converter.sink),
                converter.source.connect(pipe_ready.sink),
                pipe_ready.source.connect(source),
            ]

# PHYRX128BAligner ---------------------------------------------------------------------------------

class PHYRX128BAligner(Module):
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(phy_layout(128))
        self.source = source = stream.Endpoint(phy_layout(128))
        self.first_dword = Signal(2)

        # # #

        dat_last = Signal(64, reset_less=True)
        be_last  = Signal(8,  reset_less=True)
        self.sync += [
            If(sink.valid & sink.ready,
                dat_last.eq(sink.dat[64:]),
                be_last.eq( sink.be[8:]),
            )
        ]

        self.submodules.fsm = fsm = FSM(reset_state="ALIGNED")
        fsm.act("ALIGNED",
            sink.connect(source, omit={"first"}),
            # If "first" on DWORD2 and "last" on the same cycle, switch to UNALIGNED.
            If(sink.valid & sink.last & sink.first & (self.first_dword == 2),
                source.be[8:].eq(0),
                If(source.ready,
                    NextState("UNALIGNED")
                )
            )
        )
        fsm.act("UNALIGNED",
            sink.connect(source, omit={"first", "dat", "be"}),
            source.dat.eq(Cat(dat_last, sink.dat)),
            source.be.eq( Cat(be_last,  sink.be)),
            # If "last" and not "first" on the same cycle, switch to ALIGNED.
            If(sink.valid & sink.last & ~sink.first,
                source.be[8:].eq(0),
                If(source.ready,
                    NextState("ALIGNED")
                )
            )
        )

# RX Datapath --------------------------------------------------------------------------------------

class PHYRXDatapath(Module):
    def __init__(self, core_data_width, pcie_data_width, clock_domain, with_aligner=False):
        self.sink   = sink   = stream.Endpoint(phy_layout(pcie_data_width))
        self.source = source = stream.Endpoint(phy_layout(core_data_width))

        # # #

        if (pcie_data_width == 128) and with_aligner:
            aligner = PHYRX128BAligner()
            aligner = ClockDomainsRenamer("pcie")(aligner)
            self.submodules.aligner = aligner
            self.comb += sink.connect(aligner.sink)
            sink = aligner.source

        if (clock_domain == "pcie") and (core_data_width == pcie_data_width):
            self.comb += sink.connect(source)
        else:
            pipe_ready = stream.PipeReady(phy_layout(core_data_width))
            pipe_ready = ClockDomainsRenamer("pcie")(pipe_ready)
            converter  = stream.StrideConverter(phy_layout(pcie_data_width), phy_layout(core_data_width))
            converter  = ClockDomainsRenamer("pcie")(converter)
            cdc        = stream.ClockDomainCrossing(
                layout          = phy_layout(core_data_width),
                cd_from         = "pcie",
                cd_to           = clock_domain,
                with_common_rst = True,
                depth           = 16,
            )
            pipe_valid = stream.PipeValid(phy_layout(core_data_width))
            pipe_valid = ClockDomainsRenamer(clock_domain)(pipe_valid)
            self.submodules += pipe_ready, converter, cdc, pipe_valid
            self.comb += [
                sink.connect(pipe_ready.sink),
                pipe_ready.source.connect(converter.sink),
                converter.source.connect(cdc.sink),
                cdc.source.connect(pipe_valid.sink),
                pipe_valid.source.connect(source),
            ]

# TX/RX Padding (TLP interfaces without byte enables) ---------------------------------------------

class PHYTXPaddingRemover(Module):
    """Remove the padding DWORDs of down-converted TLPs.

    Down-converting a TLP to DWORDs outputs the unused part of its last beat as DWORDs with be == 0.
    TLP interfaces without byte enables (e.g. Lattice vc_tx_*) would send them as part of the TLP:
    drop them and move last to the last real DWORD (one DWORD of latency).
    """
    def __init__(self, data_width=32):
        self.sink   = sink   = stream.Endpoint(phy_layout(data_width))
        self.source = source = stream.Endpoint(phy_layout(data_width))

        # # #

        held_valid = Signal()
        held       = stream.Endpoint(phy_layout(data_width))
        drop       = Signal()
        self.comb += [
            drop.eq(sink.valid & (sink.be == 0)),
            source.valid.eq(held_valid & (held.last | (sink.valid & (~drop | sink.last)))),
            source.first.eq(held.first),
            source.last.eq(held.last | (drop & sink.last)),
            source.dat.eq(held.dat),
            source.be.eq(held.be),
            If(drop & ~sink.last,
                sink.ready.eq(1),
            ).Else(
                sink.ready.eq(~held_valid | source.ready),
            )
        ]
        self.sync += [
            If(source.valid & source.ready,
                held_valid.eq(0)
            ),
            If(sink.valid & sink.ready & ~drop,
                held_valid.eq(1),
                held.first.eq(sink.first),
                held.last.eq(sink.last),
                held.dat.eq(sink.dat),
                held.be.eq(sink.be),
            )
        ]

class PHYRXTLPTrimmer(Module):
    """Trim received TLPs to the DWORD count given by their header.

    For DWORD interfaces with TLP byte 0 on bits 7:0 (e.g. Lattice vc_rx_*). The Lattice PCIe IP user
    guides describe a trash DWORD appended to TLPs on the receive interface (not seen with the single-
    lane LFD2NX IP); DWORDs after the header + payload length (including an ECRC, unused by LitePCIe)
    are dropped and last is moved to the last TLP DWORD. dropped pulses for each dropped DWORD.
    """
    def __init__(self, data_width=32):
        assert data_width == 32
        self.sink    = sink   = stream.Endpoint(phy_layout(data_width))
        self.source  = source = stream.Endpoint(phy_layout(data_width))
        self.dropped = Signal()

        # # #

        fmt           = sink.dat[5:8]
        length        = Cat(sink.dat[24:32], sink.dat[16:18]) # TLP byte 3 + byte 2 bits 1:0.
        header_dwords = Signal(3)
        data_dwords   = Signal(11)
        self.comb += [
            header_dwords.eq(Mux(fmt[0], 4, 3)),
            data_dwords.eq(Mux(fmt[1], Mux(length == 0, 1024, length), 0)),
        ]

        remaining = Signal(11)  # DWORDs left in the current TLP after the current one.
        dropping  = Signal()    # Dropping DWORDs until the IP's end of packet.
        current_remaining = Signal(11)
        self.comb += [
            If(sink.first,
                current_remaining.eq(header_dwords + data_dwords - 1)
            ).Else(
                current_remaining.eq(remaining)
            ),
            If(dropping & ~sink.first,
                sink.ready.eq(1),
                self.dropped.eq(sink.valid),
            ).Else(
                sink.connect(source, omit={"last"}),
                source.last.eq(sink.last | (current_remaining == 0)),
            )
        ]
        self.sync += If(sink.valid & sink.ready,
            If(dropping & ~sink.first,
                If(sink.last, dropping.eq(0))
            ).Else(
                remaining.eq(current_remaining - 1),
                dropping.eq((current_remaining == 0) & ~sink.last),
            )
        )

class PHYRXPaddingInserter(Module):
    """Pad TLPs to a multiple of ratio DWORDs before up-conversion.

    TLP interfaces without byte enables (e.g. Lattice vc_rx_*) mark all DWORDs valid; when a TLP ends
    before the end of a beat, up-conversion would otherwise fill the rest of the beat with stale
    data and byte enables. Append DWORDs with be == 0 instead.
    """
    def __init__(self, data_width=32, ratio=2):
        self.sink   = sink   = stream.Endpoint(phy_layout(data_width))
        self.source = source = stream.Endpoint(phy_layout(data_width))

        # # #

        count     = Signal(max=max(ratio, 2))
        last_word = Signal()
        pad       = Signal()
        self.comb += [
            last_word.eq(count == (ratio - 1)),
            If(pad,
                source.valid.eq(1),
                source.last.eq(last_word),
            ).Else(
                sink.connect(source, omit={"last"}),
                source.last.eq(sink.last & last_word),
            )
        ]
        self.sync += If(source.valid & source.ready,
            If(last_word,
                count.eq(0),
            ).Else(
                count.eq(count + 1),
            ),
            If(pad,
                If(last_word, pad.eq(0))
            ).Elif(sink.last & ~last_word,
                pad.eq(1)
            )
        )

# LTSSMTracer --------------------------------------------------------------------------------------

class LTSSMTracer(Module, AutoCSR):
    def __init__(self, ltssm):
        self._history = CSRStatus(description="History of LTSSM states",
            fields = [
                CSRField("new",   offset= 0, size=6, description="New LTSSM state"),
                CSRField("old",   offset= 6, size=6, description="Old LTSSM state"),
                CSRField("ovfl",  offset=30, size=1, description="Overflow"),
                CSRField("valid", offset=31, size=1, description="Is data valid"),
        ])

        # The ltssm state signal input is sampled in the sys domain just using a MultiReg. This means
        # on change we could have an invalid state during 1 cycle.
        #
        # We also don't use an AsyncFIFO because the pcie clock domain is held in reset most of the
        # LTSSM initial negotiation defeating the point of this module.

        fifo = stream.SyncFIFO([("new", 6), ("old", 6), ("ovfl", 1)], 128)
        self.submodules += fifo

        ltssm_cur = Signal(6)
        ltssm_d1  = Signal(6)
        ltssm_d2  = Signal(6)

        overflow  = Signal()
        change    = Signal()

        self.sync += [
            ltssm_d1.eq(ltssm_cur),
            If(fifo.sink.ready, ltssm_d2.eq(ltssm_d1)),
            change.eq((ltssm_cur != ltssm_d1) & ~change),
            overflow.eq((overflow | change) & ~fifo.sink.ready),
        ]

        self.comb += [
            ltssm_cur.eq(ltssm),
            fifo.sink.new.eq(ltssm_cur),
            fifo.sink.old.eq(ltssm_d2),
            fifo.sink.ovfl.eq(overflow),
            fifo.sink.valid.eq(overflow | change),
            self._history.fields.new.eq(fifo.source.new),
            self._history.fields.old.eq(fifo.source.old),
            self._history.fields.ovfl.eq(fifo.source.ovfl),
            self._history.fields.valid.eq(fifo.source.valid),
            fifo.source.ready.eq(self._history.rd_stb),
        ]
