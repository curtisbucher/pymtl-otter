from pymtl3 import Component, InPort, OutPort, update_ff

class ProgCount( Component ):
    def construct(s):
        s.PC_LD = InPort()
        s.PC_DIN = InPort(32)
        s.PC_COUNT = OutPort(32)

        @update_ff
        def inc_pc():
            if s.reset == 0b1:
                s.PC_COUNT <<= -4
            elif s.PC_LD == 0b1:
                s.PC_COUNT <<= s.PC_DIN