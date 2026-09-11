#
# This file is part of LitePCIe.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *
from migen.sim import run_simulation, passive

from litepcie.phy.lfcpnxpciephy import LFCPNXUCFGIDReader

# Tests --------------------------------------------------------------------------------------------

class TestLFCPNXUCFGIDReader(unittest.TestCase):
    def test_id_polled_from_ucfg_0x5f(self):
        dut      = LFCPNXUCFGIDReader(poll_period=16)
        requests = []
        ucfg_id  = {"value": 0x0000}

        @passive
        def ucfg_model():
            # Accept a request after 2 cycles, complete the read 3 cycles later.
            while True:
                yield dut.ready.eq(0)
                if (yield dut.valid):
                    for _ in range(2):
                        yield
                    yield dut.ready.eq(1)
                    requests.append((yield dut.addr))
                    yield
                    yield dut.ready.eq(0)
                    for _ in range(3):
                        yield
                    yield dut.rd_data.eq(0xdead0000 | ucfg_id["value"])
                    yield dut.rd_done.eq(1)
                    yield
                    yield dut.rd_done.eq(0)
                yield

        def generator():
            # No reads before the Transaction Layer is up.
            for _ in range(64):
                yield
            self.assertEqual(requests, [])

            # Polled once link is up, and follows (re-)enumeration.
            yield dut.tl_link_up.eq(1)
            ucfg_id["value"] = 0x1700
            for _ in range(64):
                yield
            self.assertTrue(len(requests) >= 2)
            self.assertTrue(all(adr == 0x5f for adr in requests))
            self.assertEqual((yield dut.id), 0x1700)

            ucfg_id["value"] = 0x0300
            for _ in range(64):
                yield
            self.assertEqual((yield dut.id), 0x0300)

        run_simulation(dut, [generator(), ucfg_model()])

if __name__ == "__main__":
    unittest.main()
