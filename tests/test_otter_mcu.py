import unittest
from pymtl3 import DefaultPassGroup, Bits
from pymtl3.passes.backends.verilog import VerilogTranslationImportPass, VerilogPlaceholderPass, VerilogVerilatorImportPass
from src.rtl.otter_mcu import OTTER_MCU
from src.common.consts import *

from os import path
from hypothesis import given, strategies as st

BITWIDTH = 32

# See Verilog debugging for a more inclusive testcase
class TestMCU(unittest.TestCase):
    def setUp(s) -> None:
        # runs before every test
        if not hasattr(s, "dut"):
            s.dut = OTTER_MCU()
            s.dut.elaborate()

            s.dut.CU_DECODER.set_metadata( VerilogTranslationImportPass.enable, True )
            s.dut.CU_DECODER.set_metadata( VerilogPlaceholderPass.top_module, 'OTTER_CU_Decoder' )
            s.dut.CU_DECODER.set_metadata( VerilogPlaceholderPass.src_file, path.dirname(__file__) + '/../src/verilog/CU_Decoder.sv' )
            s.dut.CU_DECODER.apply(VerilogPlaceholderPass())

            s.dut.ALU.set_metadata( VerilogTranslationImportPass.enable, True )
            s.dut.ALU.set_metadata( VerilogPlaceholderPass.top_module, 'OTTER_ALU' )
            s.dut.ALU.set_metadata( VerilogPlaceholderPass.src_file, path.dirname(__file__) + '/../src/verilog/ArithLogicUnit.sv' )
            s.dut.ALU.apply(VerilogPlaceholderPass())

            s.dut.memory.set_metadata( VerilogTranslationImportPass.enable, True )
            s.dut.memory.set_metadata( VerilogPlaceholderPass.top_module, 'OTTER_mem_byte' )
            s.dut.memory.set_metadata( VerilogPlaceholderPass.src_file, path.dirname(__file__) + '/../src/verilog/bram_dualport.sv' )
            s.dut.memory.set_metadata( VerilogPlaceholderPass.v_include, [path.dirname(__file__) + '/../src/rtl'] )
            s.dut.memory.apply(VerilogPlaceholderPass())

            s.dut = VerilogTranslationImportPass()( s.dut )

            s.dut.apply(DefaultPassGroup(textwave=False, linetrace=True))
        s.dut.sim_reset()

    def tearDown(s) -> None:
        # runs after every test
        if s.dut.sim_cycle_count():
            print("final:", s.dut.line_trace())

    def test_init(s):
        s.dut.sim_reset()
        for _ in range(100):
            s.dut.sim_tick()
        assert False

if __name__ == "__main__":
    unittest.main()
    suite = unittest.TestLoader().loadTestsFromTestCase(TestMCU)
    unittest.TextTestRunner(verbosity=2).run(suite)
