import unittest
from pymtl3 import DefaultPassGroup, Bits
from pymtl3.passes.backends.verilog import VerilogTranslationImportPass, VerilogPlaceholderPass
from src.rtl.otter_mcu import OTTER_MCU
from src.common.consts import *

from os import path
from hypothesis import given, strategies as st

BITWIDTH = 32

# See Verilog debugging for a more inclusive testcase
class TestALU(unittest.TestCase):
    def setUp(s) -> None:
        # runs before every test
        if not hasattr(s, "dut"):
            s.dut = OTTER_MCU()
            s.dut.elaborate()

            s.dut.set_metadata( VerilogTranslationImportPass.enable, True )
            s.dut.apply(VerilogPlaceholderPass())
            s.dut = VerilogTranslationImportPass()( s.dut )

            s.dut.apply(DefaultPassGroup(textwave=False, linetrace=True))
        s.dut.sim_reset()

    def tearDown(s) -> None:
        # runs after every test
        if s.dut.sim_cycle_count():
            print("final:", s.dut.line_trace())

    def test_init(s):
        pass

if __name__ == "__main__":
    unittest.main()
    suite = unittest.TestLoader().loadTestsFromTestCase(TestALU)
    unittest.TextTestRunner(verbosity=2).run(suite)
