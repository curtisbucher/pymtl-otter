from pymtl3 import mk_bitstruct, mk_bits

opcode_t = mk_bits( 7 )

instr_t = mk_bitstruct("instr_t", {
    "opcode": opcode_t,
    "rs1": mk_bits(5),
    "rs2": mk_bits(5),
    "rd": mk_bits(5),
    "rs1_used": mk_bits(1),
    "rs2_used": mk_bits(1),
    "rd_used": mk_bits(1),
    "alu_fun": mk_bits(4),
    "memWrite": mk_bits(1),
    "memRead2": mk_bits(1),
    "regWrite": mk_bits(1),
    "rf_wr_sel": mk_bits(2),
    "func3": mk_bits(3),
    "pc": mk_bits(32)}
)

# OPCODES
LUI = 0b0110111
AUIPC = 0b0010111
JAL = 0b1101111
JALR = 0b1100111
BRANCH = 0b1100011
LOAD = 0b0000011
STORE = 0b0100011
OP_IMM = 0b0010011
OP = 0b0110011
SYSTEM = 0b0001111

# for printing
opcodes = {
    0: "NULL",
    0b0110111 : "LUI",
    0b0010111 : "AUIPC",
    0b1101111 : "JAL",
    0b1100111 : "JALR",
    0b1100011 : "BRANCH",
    0b0000011 : "LOAD",
    0b0100011 : "STORE",
    0b0010011 : "OP_IMM",
    0b0110011 : "OP",
    0b0001111 : "SYSTEM",
}

# ALU FUNCTIONS
ALU_ADD = 0b0000
ALU_SUB = 0b1000
ALU_OR = 0b0110
ALU_AND = 0b0111
ALU_XOR = 0b0100
ALU_SRL = 0b0101
ALU_SLL = 0b0001
ALU_SRA = 0b1101
ALU_SLT = 0b0010
ALU_SLTU = 0b0011
ALU_LUI_COPY = 0b1001
ALU_MUL = 0b1010

# for printing
alu_funs = {
    0b0000 : "ALU_ADD",
    0b1000 : "ALU_SUB",
    0b0110 : "ALU_OR",
    0b0111 : "ALU_AND",
    0b0100 : "ALU_XOR",
    0b0101 : "ALU_SRL",
    0b0001 : "ALU_SLL",
    0b1101 : "ALU_SRA",
    0b0010 : "ALU_SLT",
    0b0011 : "ALU_SLTU",
    0b1001 : "ALU_LUI_COPY",
    0b1010 : "ALU_MUL",
}