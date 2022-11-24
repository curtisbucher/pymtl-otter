from pymtl3 import Component, Placeholder, InPort, OutPort

class OTTER_CU_Decoder(Component, Placeholder):
    def construct( s ):
        s.CU_OPCODE = InPort(7)
        s.CU_FUNC3 = InPort(3)
        s.CU_FUNC7 = InPort(7)
        s.CU_BR_EQ = InPort(1)
        s.CU_BR_LT = InPort(1)
        s.CU_BR_LTU = InPort(1)
        s.CU_ALU_SRCA = OutPort(1)
        s.CU_ALU_SRCB = OutPort(2)
        s.CU_ALU_FUN = OutPort(4)
        s.CU_RF_WR_SEL = OutPort(2)

