from pymtl3 import Component, InPort, OutPort, update_ff

class ProgCount( Component ):
    def construct(s):
        s.PC_CLK = InPort()
        s.PC_RST = InPort()
        s.PC_LD = InPort()
        s.PC_DIN = InPort()
        s.PC_COUNT = OutPort()

        @update_ff
        def inc_pc():
            if s.PC_RST == 0b1:
                s.PC_COUNT <<= 0
            elif s.PC_LD == 0b1:
                s.PC_COUNT <<= s.PC_DIN