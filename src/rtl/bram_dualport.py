from pymtl3 import Component, Placeholder, InPort, OutPort

class OTTER_mem_dualport(Component, Placeholder):
    def construct( s ):
        s.MEM_CLK = InPort()
        s.MEM_ADDR1 = InPort(32)    #Instruction Memory Port
        s.MEM_ADDR2 = InPort(32)    #Data Memory Port
        s.MEM_DIN2 = InPort(32)
        s.MEM_WRITE2 = InPort(32)
        s.MEM_READ1 = InPort(32)
        s.MEM_READ2 = InPort(32)
        s.IO_IN = InPort(32)
        s.ERR = OutPort()
        # s.MEM_SIZE = InPort(2)
        s.MEM_SIGN = InPort()
        s.MEM_DOUT1 = OutPort(32)
        s.MEM_DOUT2 = OutPort(32)
        s.IO_WR = OutPort()

class OTTER_mem_byte(Component, Placeholder):
    def construct( s ):
        s.MEM_CLK = InPort()
        s.MEM_ADDR1 = InPort(32)    #Instruction Memory Port
        s.MEM_ADDR2 = InPort(32)    #Data Memory Port
        s.MEM_DIN2 = InPort(32)
        s.MEM_WRITE2 = InPort(32)
        s.MEM_READ1 = InPort(32)
        s.MEM_READ2 = InPort(32)
        s.IO_IN = InPort(32)
        s.ERR = OutPort()
        s.MEM_SIZE = InPort(2)
        s.MEM_SIGN = InPort()
        s.MEM_DOUT1 = OutPort(32)
        s.MEM_DOUT2 = OutPort(32)
        s.IO_WR = OutPort()



