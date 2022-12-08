# Pymtl Otter Implementation (CPE333 Final Project)
An implementation of Cal Poly's RISC-V OTTER written in [PyMTL3](https://github.com/pymtl/pymtl3), an open-source Python-based hardware generation, simulation, and verification framework

## Quickstart
Download Repo: \
`$ git clone https://github.com/curtisbucher/pymtl-otter`

Install Requirements: \
`$ pip install -r requirements.txt`

Install [Verilator](https://www.veripool.org/verilator/) (Mac): \
`$ brew install verilator`

Install [Verilator](https://www.veripool.org/verilator/) (Ubuntu): \
`$ apt-get install verilator`

## Testing
Run test suite with pymtl: \
`$ make test` or \
`$ pytest tests/*`

Run test suite with verilator: \
`$ make test-verilog` or \
`$ pytest tests/* --test-verilog`

## Synthesis (*not currently working*)
Compile PyMTL code into SystemVerilog, under `translated/`: \
`$ make synthesis` or \
`$ python synthesis.py`

## Design
### Project Diagram
![Project Diagram](https://raw.githubusercontent.com/curtisbucher/pymtl-otter/main/docs/FinalProjectDiagram.png)
### Architecture Diagram
![Architecture Diagram](https://raw.githubusercontent.com/curtisbucher/pymtl-otter/main/docs/Pipelined%20Arch%20Diagram.png)

## Results
When running tests/test_otter_mcu.py, the processor is put through Dr.Callenes' RISCV *testAll_noHaz.mem* program. It completes the program successfully, with identical performance to the reference pipelined otter design. While the processor is built with forwarding and stalling to alleviate data and control hazards, there were still some bugs in running *testAll_Haz.mem* that will be the subject of future work.

## Future Work
* Fix forwarding and stalling.
* Replace naive memory module with synthesizeable memory module that is capable of simulating cache misses.
* Synthesize code into SystemVerilog.
* Implement more robust testing using the Spike RISCV ISA Simulator.

## Resources
[Class Presentation](https://github.com/curtisbucher/pymtl-otter/raw/main/docs/FinalProjectPresentation.pdf)
