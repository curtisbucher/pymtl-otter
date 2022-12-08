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
            s.dut = OTTER_MCU(mem_file="/home/cubucher/Desktop/pymtl-otter/src/mem/testAll_noHaz_mod.mem")
            s.dut.elaborate()

            s.dut.CU_DECODER.set_metadata( VerilogTranslationImportPass.enable, True )
            s.dut.CU_DECODER.set_metadata( VerilogPlaceholderPass.top_module, 'OTTER_CU_Decoder' )
            s.dut.CU_DECODER.set_metadata( VerilogPlaceholderPass.src_file, path.dirname(__file__) + '/../src/verilog/CU_Decoder.sv' )
            s.dut.CU_DECODER.apply(VerilogPlaceholderPass())

            s.dut.ALU.set_metadata( VerilogTranslationImportPass.enable, True )
            s.dut.ALU.set_metadata( VerilogPlaceholderPass.top_module, 'OTTER_ALU' )
            s.dut.ALU.set_metadata( VerilogPlaceholderPass.src_file, path.dirname(__file__) + '/../src/verilog/ArithLogicUnit.sv' )
            s.dut.ALU.apply(VerilogPlaceholderPass())

            # s.dut.memory.set_metadata( VerilogTranslationImportPass.enable, True )
            # s.dut.memory.set_metadata( VerilogPlaceholderPass.top_module, 'OTTER_mem_byte' )
            # s.dut.memory.set_metadata( VerilogPlaceholderPass.src_file, path.dirname(__file__) + '/../src/verilog/bram_dualport.sv' )
            # s.dut.memory.set_metadata( VerilogPlaceholderPass.v_include, [path.dirname(__file__) + '/../src/rtl'] )
            s.dut.memory.apply(VerilogPlaceholderPass())

            s.dut = VerilogTranslationImportPass()( s.dut )

            s.dut.apply(DefaultPassGroup(textwave=True, linetrace=True, vcdwave="vcd/test_mcu"))

        s.dut.sim_reset()
        for i, x in enumerate(s.dut.memory.mem):
            print(f"{hex(i*4):4} : {x}")

    def tearDown(s) -> None:
        # runs after every test
        if s.dut.sim_cycle_count():
            print("final:", s.dut.line_trace())

        # print registers
        print("\nRegisters:")
        reg_names = ["zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"]
        for r, n in zip(s.dut.RF.regs, reg_names):
            print(f"{n:4} : {r}")

    def test_testall(s):
        """Test all instructions, with testall.mem"""
        FAIL_ADDR = 0x4b4
        s.dut.sim_reset()

        # Getting program started, so PC > 0
        while s.dut.pc_value < 5:
            s.dut.sim_tick()

        # Run until testall fails or program restarts (success)
        while s.dut.pc_value > 5 and s.dut.pc_value < 0x500:
            s.dut.sim_tick()
            # goes to FAIL_ADDR when testall is fails
            assert s.dut.pc_value != FAIL_ADDR, "Testall failed at cycle {}".format(s.dut.sim_cycle_count())


if __name__ == "__main__":
    unittest.main()
    suite = unittest.TestLoader().loadTestsFromTestCase(TestMCU)
    unittest.TextTestRunner(verbosity=2).run(suite)
