# Pymtl Otter Implementation (CPE333 Final Project)
An implementation of Cal Poly's RISC-V OTTER written in [PyMTL3](https://github.com/pymtl/pymtl3), an open source high level hardware developement framework.

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
![Diagram](https://raw.githubusercontent.com/curtisbucher/pymtl-otter/main/docs/FinalProjectDiagram.png)
### Architecture Diagram
![Diagram](https://raw.githubusercontent.com/curtisbucher/pymtl-otter/main/docs/Pipelined%20Arch%20Diagram.png)

## Resources
[Presentation](https://github.com/curtisbucher/pymtl-otter/raw/main/docs/FinalProjectPresentation.pdf)
