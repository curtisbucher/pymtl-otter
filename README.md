# Pymtl Otter Implementation (CPE333 Final Project)
An implementation of Cal Poly's RISC-V OTTER written in PyMTL3, an open source high level hardware developement framework.

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

## Resources
[Presentation](https://github.com/Cal-Poly-RAMP/ramp-core/blob/9185850a935498318f1f76ee783e12f6f9012397/docs/source/Bucher,%20Callenes%20Poster.pdf) \
[Architecture Diagram](https://github.com/Cal-Poly-RAMP/ramp-core/blob/9185850a935498318f1f76ee783e12f6f9012397/docs/source/ramp-high-level-interface-printing.drawio.pdf)
