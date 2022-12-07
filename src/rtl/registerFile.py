from pymtl3 import Component, InPort, OutPort, update, update_ff

class RegisterFile(Component):
    def construct( s ):
        s.Read1 = InPort(5)
        s.Read2 = InPort(5)
        s.WriteReg = InPort(5)
        s.WriteData = InPort(32)
        s.RegWrite = InPort(1)
        s.clock = InPort(1)
        s.Data1 = OutPort(32)
        s.Data2 = OutPort(32)
        s.regs = [OutPort(32) for _ in range(32)]

        @update
        def read():
            # Default to zero if reading from zero register
            s.Data1 @= 0
            s.Data2 @= 0

            if s.Read1:
                s.Data1 @= s.regs[s.Read1] if s.Read1 != s.WriteReg else s.WriteData
            if s.Read2:
                s.Data2 @= s.regs[s.Read2] if s.Read2 != s.WriteReg else s.WriteData

        @update_ff
        def write():
            if s.RegWrite and s.WriteReg:
                s.regs[s.WriteReg] <<= s.WriteData



