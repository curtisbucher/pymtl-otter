from pymtl3 import (
    Component,
    InPort,
    OutPort,
    clog2,
    update,
    update_ff,
    Wire,
    mk_bits,
    Bits1,
    Bits32,
    update_once,
)

# TODO: make byte addressable

class Memory(Component):
    def construct(
        s, Type, num_entries=32, rd_ports=1, wr_ports=1, reset_value=0, file=None
    ):
        addr_type = mk_bits(max(1, clog2(num_entries)))

        s.raddr = [InPort(addr_type) for _ in range(rd_ports)]
        s.rdata = [OutPort(Type) for _ in range(rd_ports)]

        s.waddr = [InPort(addr_type) for _ in range(wr_ports)]
        s.wdata = [InPort(Type) for _ in range(wr_ports)]
        s.wen = [InPort(Bits1) for _ in range(wr_ports)]

        s.mem_read1 = InPort()
        s.mem_read2 = InPort()

        s.mem_size = InPort(2)
        s.sign = InPort(1)


        s.mem = [Wire(Type) for _ in range(num_entries)]

        # for byte addressable memory
        addr_shift = clog2(Type.nbits >> 3)

        # for latching data on mem_read1, mem_read2
        s.rdata1_latch = Wire(Type)
        s.rdata2_latch = Wire(Type)

        @update_ff
        def up_rf_write():
            if s.reset:
                # loading instruction memory from file
                i = 0
                if file:
                    with open(file, "r") as f:
                        for i, line in enumerate(f):
                            s.mem[i] <<= int(line, 16)
                for e in range(i + 1, num_entries):
                    s.mem[e] <<= reset_value
            else:
                for i in range(wr_ports):
                    if s.wen[i]:
                        # TODO: CL debugging
                        # assert not (s.waddr[i] % (Type.nbits // 8)), f"Address must be {Type.nbits // 8}-byte aligned"
                        s.mem[s.waddr[i] >> addr_shift] <<= s.wdata[i]

        @update_ff
        def up_rf_read_latch():
            if(s.mem_read1):
                s.rdata1_latch <<= s.mem[s.raddr[0] >> addr_shift]
            if(s.mem_read2):
                s.rdata2_latch <<= s.mem[s.raddr[1] >> addr_shift]

        @update
        def up_rf_read():
            # for i in range(rd_ports):
            #     # TODO: CL debugging
            #     # assert not (s.raddr[i] % (Type.nbits // 8)), f"Address must be {Type.nbits // 8}-byte aligned"
            #     # byte addressable
            #     if(s.mem_read1):
            #         s.rdata[i] @= s.mem[s.raddr[i] >> addr_shift]
            #     s.rdata[i] @= s.mem[s.raddr[i] >> addr_shift]
            if(s.mem_read1):
                s.rdata[0] @= s.mem[s.raddr[0] >> addr_shift]
            else:
                s.rdata[0] @=s.rdata1_latch
            if(s.mem_read2):
                s.rdata[1] @= s.mem[s.raddr[1] >> addr_shift]
            else:
                s.rdata[1] @=s.rdata2_latch

    def line_trace(s):
        nshown = min(32, len(s.mem))
        more = len(s.mem) - nshown
        return f"DRAM: {[hex(e.uint()) for e in s.mem[0:nshown]]}(+{more} more)"