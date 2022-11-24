from pymtl3 import Component, Placeholder, InPort, OutPort

class OTTER_registerFile(Component, Placeholder):
    def construct( s ):
        s.Read1 = InPort(5)
        s.Read2 = InPort(5)
        s.WriteReg = InPort(5)
        s.WriteData = InPort(32)
        s.RegWrite = InPort(1)
        s.clock = InPort(1)
        s.Data1 = OutPort(32)
        s.Data2 = OutPort(32)
        s.RF = [OutPort(32) for _ in range(32)]