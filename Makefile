# --cov-report=html, -n auto
test:
	pytest -s tests/ --cov=src --cov-branch --cov-report=term-missing --cov-fail-under=90; rm -r *_noparam*;

# not working rn, only works with io from ramp-core
test-verilog:
	pytest -s tests/ --test-verilog --cov=src --cov-branch --cov-report=term-missing --cov-fail-under=90; rm -r *_noparam*;

# Test in Python
format:
	black src/*/*.py tests/*.py

synthesis:
	python synthesize.py

#removing translated files, logfiles.
clean: clean-verilator
	rm translated/* ; rm vcd/* ;

# cleaning up verilator files
clean-verilator:
	rm -r *_noparam*;

.PHONY: test test-verilog synthesis
