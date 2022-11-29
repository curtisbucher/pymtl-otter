## TODO: REMEMBER THAT ALL SLICES WILL HABE TO BE CHANGED FROM VERILOG TO PYMTL.
## DIFFERENCE IN INCLUSIVE / EXCLUSIVE SLICING
## AND, OR, NOT may need to be replaced with &, |, ~
from pymtl3 import Component, Wire, Bits, InPort, OutPort, update, update_ff, concat, sext
from pymtl3.stdlib.basic_rtl.register_files import RegisterFile

from src.rtl.ProgCount import ProgCount
from src.rtl.CU_Decoder import OTTER_CU_Decoder
from src.rtl.registerFile import OTTER_registerFile
from src.rtl.OTTER_ALU import OTTER_ALU
from src.rtl.bram_dualport import OTTER_mem_byte
from src.common.util import *
from src.common.consts import *

class OTTER_MCU(Component):
    def construct(s):
        s.INTR = InPort(1)
        s.IOBUS_IN = InPort(32)
        s.IOBUS_OUT = OutPort(32)
        s.IOBUS_ADDR = OutPort(32)
        s.IOBUS_WR = OutPort(1)

        s.opcode = Wire(7)
        s.pc = Wire(32)
        s.pc_value = Wire(32)
        s.next_pc = Wire(32)
        s.jalr_pc = Wire(32)
        s.branch_pc = Wire(32)
        s.jump_pc = Wire(32)
        s.int_pc = Wire(32)
        s.A = Wire(32)
        s.B = Wire(32)
        s.I_immed = Wire(32)
        s.S_immed = Wire(32)
        s.U_immed = Wire(32)
        s.aluBin = Wire(32)
        s.aluAin = Wire(32)
        s.aluResult = Wire(32)
        s.rfIn = Wire(32)
        s.mem_data = Wire(32)
        s.B_immed = Wire(32)
        s.J_immed = Wire(32)

        s.IR = Wire(32)
        s.memRead1 = Wire(1)
        s.memRead2 = Wire(1)

        s.pcWrite = Wire(1)
        s.regWrite = Wire(1)
        s.memWrite = Wire(1)
        s.op1_sel = Wire(1)
        s.mem_op = Wire(1)
        s.memRead = Wire(1)
        s.opB_sel = Wire(2)
        s.rf_sel = Wire(2)
        s.wb_sel = Wire(2)
        s.pc_sel = Wire(2)
        s.alu_fun = Wire(4)
        s.opA_sel = Wire(1)

        s.br_lt = Wire(1)
        s.br_eq = Wire(1)
        s.br_ltu = Wire(1)
        s.wb_enable = Wire(1)

        s.stall_pc = Wire(1)
        s.stall_if = Wire(1)
        s.stall_de = Wire(1)
        s.stall_ex = Wire(1)
        s.stall_mem = Wire(1)
        s.stall_wb = Wire(1)

        s.if_de_invalid = Wire(1)
        s.de_ex_invalid = Wire(1)
        s.ex_mem_invalid = Wire(1)
        s.mem_wb_invalid = Wire(1)

        # Pipeline Registers --------------------------------------------
        s.mem_wb_inst = Wire(instr_t)

        # Instruction Fetch ----------------------------------------------------
        s.if_de_pc = Wire(32)

        @update_ff
        def up_if_de_pc():
            if ~s.stall_if:
                s.if_de_pc <<= s.pc

        s.pcWrite = not s.stall_pc
        s.memRead1 = not s.stall_if

        s.opcode //= s.IR[0:7] # opcode shortcut
        # PC is byte-addressed but our memory is word addressed
        s.PC = ProgCount()
        s.PC.PC_LD //= s.pcWrite
        s.PC.PC_DIN //= s.pc_value
        s.PC.PC_COUNT //= s.pc

        # Creates a 2-to-1 multiplexor used to select the source of the next PC
        @update
        def pc_data_src():
            # pc target calculations
            s.next_pc @= s.pc + 4    # PC is byte aligned, memory is word aligned

            if s.pc_sel == 0:
                s.pc_value @= s.next_pc
            elif s.pc_sel == 1:
                s.pc_value @= s.jalr_pc
            elif s.pc_sel == 2:
                s.pc_value @= s.branch_pc
            elif s.pc_sel == 3:
                s.pc_value @= s.jump_pc

        # Instruction Decode ---------------------------------------------------

        s.de_ex_opA = Wire(32)
        s.de_ex_opB = Wire(32)
        s.de_ex_rs2 = Wire(32)
        s.de_ex_I_immed = Wire(32)
        s.de_ex_J_immed = Wire(32)
        s.de_ex_B_immed = Wire(32)

        s.de_ex_inst = Wire(instr_t)
        s.de_inst = Wire(instr_t)

        s.de_inst.rs1 //= s.IR[15:20]
        s.de_inst.rs2 //= s.IR[20:25]
        s.de_inst.rd //= s.IR[7:12]
        s.de_inst.opcode //= s.opcode
        s.de_inst.alu_fun //= s.alu_fun
        s.de_inst.rf_wr_sel //= s.wb_sel
        s.de_inst.pc //= s.if_de_pc
        s.de_inst.func3 //= s.IR[12:15]

        s.de_inst.rs1_used //= (s.de_inst.rs1 != 0 and
                                s.de_inst.opcode != LUI and
                                s.de_inst.opcode != AUIPC and
                                s.de_inst.opcode != JAL)

        s.de_inst.rs2_used //= (s.de_inst.rs2 != 0 and
                                (s.de_inst.opcode == BRANCH or
                                s.de_inst.opcode == STORE or
                                s.de_inst.opcode == OP))

        s.de_inst.rd_used //= (s.de_inst.opcode != BRANCH and
                                s.de_inst.opcode != STORE)

        s.de_inst.regWrite //= (s.de_inst.opcode != BRANCH and
                                s.de_inst.opcode != STORE)

        s.de_inst.memWrite //= (s.de_inst.opcode == STORE)
        s.de_inst.memRead2 //= (s.de_inst.opcode == LOAD)

        s.CU_DECODER = OTTER_CU_Decoder()
        s.CU_DECODER.CU_OPCODE //= s.opcode
        s.CU_DECODER.CU_FUNC3 //= s.IR[12:15]
        s.CU_DECODER.CU_FUNC7 //= s.IR[25:32]
        s.CU_DECODER.CU_BR_EQ //= s.br_eq
        s.CU_DECODER.CU_BR_LT //= s.br_lt
        s.CU_DECODER.CU_BR_LTU //= s.br_ltu
        s.CU_DECODER.CU_ALU_SRCA //= s.opA_sel
        s.CU_DECODER.CU_ALU_SRCB //= s.opB_sel
        s.CU_DECODER.CU_ALU_FUN //= s.alu_fun
        s.CU_DECODER.CU_RF_WR_SEL //= s.wb_sel

        # Selecting inputs to ALU
        @update
        def sel_alu_inputs():
            # input A
            if s.opA_sel == 0:
                s.aluAin @= s.A
            elif s.opA_sel == 1:
                s.aluAin @= s.U_immed
            # input B
            if s.opB_sel == 0:
                s.aluBin @= s.B
            elif s.opB_sel == 1:
                s.aluBin @= s.I_immed
            elif s.opB_sel == 2:
                s.aluBin @= s.S_immed
            elif s.opB_sel == 3:
                s.aluBin @= s.de_inst.pc

        # Create RISC-V register file
        # s.RF = OTTER_registerFile()
        # s.RF.rf_rs1 //= s.de_inst.rs1
        # s.RF.rf_rs2 //= s.de_inst.rs2
        # s.RF.rf_rd //= s.mem_wb_inst.rd
        # s.RF.rf_in //= s.rfIn
        # s.RF.rf_wb_enable //= s.wb_enable
        # s.RF.rf_outA //= s.A
        # s.RF.rf_outB //= s.B
        # s.RF.rf_clk //= s.clk
        s.RF = RegisterFile(
            mk_bits(32), nregs=32, rd_ports=2, wr_ports=1, const_zero=True
        )
        s.RF.raddr[0] //= s.de_inst.rs1
        s.RF.raddr[1] //= s.de_inst.rs2
        s.RF.waddr[0] //= s.mem_wb_inst.rd
        s.RF.wdata[0] //= s.rfIn
        s.RF.wen[0] //= s.wb_enable
        s.RF.rdata[0] //= s.A
        s.RF.rdata[1] //= s.B

        # generate immediates
        @update
        def gen_immed():
            # s.S_immed = concat(sext(s.IR[31], 20), s.IR[25:32], s.IR[7:12])
            tmp1 = concat(s.IR[25:32], s.IR[7:12])  # concat can't be nested in sext
            s.S_immed @= sext(tmp1, 32)
            s.I_immed @= concat(sext(s.IR[31], 20), s.IR[20:32])
            s.U_immed @= concat(s.IR[12:32], sext(0, 12))
            s.B_immed @= concat(
                sext(s.IR[31], 20),
                s.IR[7],
                s.IR[25:31],
                s.IR[8:12],
                sext(0, 1)
            )
            s.J_immed @= concat(
                sext(s.IR[31], 12),
                s.IR[12:20],
                s.IR[20],
                s.IR[21:31],
                sext(0, 1)
            )

        @update_ff
        def assign_de_ex():
            if not s.stall_de:
                s.de_ex_inst <<= s.de_inst
                s.de_ex_opA <<= s.aluAin
                s.de_ex_opB <<= s.aluBin
                s.de_ex_rs2 <<= s.B
                s.de_ex_I_immed <<= s.I_immed
                s.de_ex_J_immed <<= s.J_immed
                s.de_ex_B_immed <<= s.B_immed

        # HAZARD DETECTION ---------------------------------------------------
        # insert 1 bubble on load-use, stall if and de

        ld_use_hazard = ((not s.if_de_invalid) and (not s.de_ex_invalid) and
                        ((s.de_ex_inst.memRead2 and
                        (s.de_inst.rs1 == s.de_ex_inst.rd and s.de_inst.rs1_used or
                        (s.de_inst.rs2 == s.de_ex_inst.rd and s.de_inst.rs2_used)))))
        s.stall_if = ld_use_hazard
        s.stall_pc = ld_use_hazard
        s.stall_de = ld_use_hazard

        # For instruction that is branch/jump, if changes the PC,
        branch_taken = ((not s.de_ex_invalid) and (s.pc_sel != 0))

        @update_ff
        def assign_if_de_invalid():
            if(s.reset):
                s.if_de_invalid <<= 1
                s.de_ex_invalid <<= 1
                s.ex_mem_invalid <<= 1
                s.mem_wb_invalid <<= 1
            else:
                if not s.stall_if:
                    s.if_de_invalid <<= branch_taken
                if not s.stall_de:
                    s.de_ex_invalid <<= s.if_de_invalid | branch_taken
                elif not s.stall_ex:
                    s.de_ex_invalid <<= 1

                if not s.stall_ex:
                    s.ex_mem_invalid <<= s.de_ex_invalid
                if not s.stall_mem:
                    s.mem_wb_invalid <<= s.ex_mem_invalid

        # Execute stage -----------------------------------------------------------
        s.ex_mem_rs2 = Wire(32)
        s.rs2_forwarded = Wire(32)
        s.ex_mem_aluRes = Wire(32)
        s.ex_mem_inst = Wire(instr_t)
        s.opA_forwarded = Wire(32)
        s.opB_forwarded = Wire(32)

        # Creating ALU
        s.ALU = OTTER_ALU()

        # Branch Condition Generator
        @update
        def branch_cond_gen():
            s.br_lt @= signed_lt(s.opA_forwarded, s.opB_forwarded)
            s.br_eq @= s.opA_forwarded == s.opB_forwarded
            s.br_ltu @= s.opA_forwarded < s.opB_forwarded

        s.brn_cond = Wire(32)
        @update
        def brn_cond():
            if s.de_ex_inst.func3 == 0b000:
                s.brn_cond @= s.br_eq
            elif s.de_ex_inst.func3 == 0b001:
                s.brn_cond @= ~s.br_eq
            elif s.de_ex_inst.func3 == 0b100:
                s.brn_cond @= s.br_lt
            elif s.de_ex_inst.func3 == 0b101:
                s.brn_cond @= ~s.br_lt
            elif s.de_ex_inst.func3 == 0b110:
                s.brn_cond @= s.br_ltu
            elif s.de_ex_inst.func3 == 0b111:
                s.brn_cond @= ~s.br_ltu
            else:
                s.brn_cond @= 0

            if(~s.de_ex_invalid):
                if s.de_ex_inst.opcode == JAL:
                    s.pc_sel @= 0b11
                elif s.de_ex_inst.opcode == JALR:
                    s.pc_sel @= 0b01
                elif s.de_ex_inst.opcode == BRANCH:
                    # s.brn_cond << 1?
                    s.pc_sel @= 0b10 if s.brn_cond else 0b00
                else:
                    s.pc_sel @= 0b00
            else:
                s.pc_sel @= 0b00

            # Calculating branches
            s.jalr_pc @= s.de_ex_I_immed + s.opA_forwarded
            s.branch_pc @= s.de_ex_B_immed + s.de_ex_inst.pc # byte aligned addresses
            s.jump_pc @= s.de_ex_J_immed + s.de_ex_inst.pc

        @update_ff
        def update_ex_mem():
            if not s.stall_ex:
                s.ex_mem_aluRes <<= s.aluResult
                s.ex_mem_inst <<= s.de_ex_inst
                s.ex_mem_rs2 <<= s.rs2_forwarded

        # Memory stage ------------------------------------------------------------
        s.mem_wb_aluRes = Wire(32)

        s.mem_enable = Wire(1)
        s.mem_Wenable = (not s.ex_mem_invalid) and s.ex_mem_inst.memWrite
        s.mem_Renable = (not s.ex_mem_invalid) and s.ex_mem_inst.memRead2

        # TODO: replace with pymtl memory modules
        s.memory = OTTER_mem_byte()
        s.memory.MEM_CLK //= s.clk
        s.memory.MEM_ADDR1 //= s.pc
        s.memory.MEM_ADDR2 //= s.ex_mem_aluRes
        s.memory.MEM_DIN2 //= s.ex_mem_rs2
        s.memory.MEM_WRITE2 //= s.mem_Wenable
        s.memory.MEM_READ1 //= s.memRead1
        s.memory.MEM_READ2 //= s.mem_Renable
        # ERR=,
        s.memory.MEM_DOUT1 //= s.IR
        s.memory.MEM_DOUT2 //= s.mem_data
        s.memory.IO_IN //= s.IOBUS_IN
        s.memory.IO_WR //= s.IOBUS_WR
        s.memory.MEM_SIZE //= s.ex_mem_inst.func3[0:2]
        s.memory.MEM_SIGN //= s.ex_mem_inst.func3[2]

        # IO
        s.IOBUS_ADDR //= s.ex_mem_aluRes
        s.IOBUS_OUT //= s.ex_mem_rs2

        @update_ff
        def update_mem_wb():
            if not s.stall_mem:
                s.mem_wb_inst <<= s.ex_mem_inst
                s.mem_wb_aluRes <<= s.ex_mem_aluRes

        # Writeback stage ----------------------------------------------------------

        s.wb_enable = (not s.stall_wb) and (not s.mem_wb_invalid) and s.mem_wb_inst.regWrite

        # creating multiplexor to select reg write back data
        @update
        def mux_rfIn():
            if s.mem_wb_inst.rf_wr_sel == 0:
                s.rfIn @= s.mem_wb_inst.pc + 4
            elif s.mem_wb_inst.rf_wr_sel == 1:
                s.rfIn @= s.csr_reg
            elif s.mem_wb_inst.rf_wr_sel == 2:
                s.rfIn @= s.mem_data
            elif s.mem_wb_inst.rf_wr_sel == 3:
                s.rfIn @= s.mem_wb_aluRes

        # Forwarding Logic ---------------------------------------------------------
        s.valid_forward_from_mem = Wire(1)
        s.valid_forward_from_wb = Wire(1)
        s.valid_forward_from_mem //= (s.ex_mem_inst.regWrite and not s.ex_mem_invalid)
        s.valid_forward_from_wb //= (s.mem_wb_inst.regWrite and not s.mem_wb_invalid)

        s.opA_forwarded //= s.de_ex_opA
        s.opB_forwarded //= s.de_ex_opB
        s.rs2_forwarded //= s.de_ex_rs2

        @update
        def mux_opA_forwarded():
            if s.valid_forward_from_mem and (s.ex_mem_inst.rd == s.de_ex_inst.rs1) and s.de_ex_inst.rs1_used:
                s.opA_forwarded = s.ex_mem_aluRes
            elif s.valid_forward_from_wb and (s.mem_wb_inst.rd == s.de_ex_inst.rs1) and s.de_ex_inst.rs1_used:
                s.opA_forwarded = s.rfIn

            if s.valid_forward_from_mem and (s.ex_mem_inst.rd == s.de_ex_inst.rs2) and s.de_ex_inst.rs2_used:
                if s.de_ex_inst.opcode != STORE:
                    s.opB_forwarded = s.ex_mem_aluRes
                else:
                    s.rs2_forwarded = s.ex_mem_aluRes
            elif s.valid_forward_from_wb and (s.mem_wb_inst.rd == s.de_ex_inst.rs2) and s.de_ex_inst.rs2_used:
                if s.de_ex_inst.opcode != STORE:
                    s.opB_forwarded = s.rfIn
                else:
                    s.rs2_forwarded = s.rfIn