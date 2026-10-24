# Simulation with Verilator (5.x)
#
#   make            lint + main test
#   make test       main random test (good + bad sentences)
#   make test_ovf   slow output + small FIFO, sentences must be dropped cleanly
#   make regress    main test with several seeds
#   make synth      open source synthesis check: sv2v + Yosys synth_gowin
#                   (the real bitstream is made with Gowin EDA, see fpga/)
#   make wave       main test with a VCD dump (tb.vcd)
#
#   SEED=5 N=5000 ERR=40 make test

VERILATOR ?= verilator
YOSYS     ?= yosys
SV2V      ?= sv2v

ROOT := ..
RTL  := $(ROOT)/rtl/nmea_pkg.sv \
        $(ROOT)/rtl/uart_rx.sv \
        $(ROOT)/rtl/uart_tx.sv \
        $(ROOT)/rtl/dec_accum.sv \
        $(ROOT)/rtl/nmea_parser.sv \
        $(ROOT)/rtl/nmea_fifo.sv \
        $(ROOT)/rtl/nmea_core.sv
TOP_FPGA := $(ROOT)/fpga/tang_nano_4k_top.sv
TB   := $(ROOT)/tb/nmea_tb_pkg.sv \
        $(ROOT)/tb/nmea_sva.sv \
        $(ROOT)/tb/tb_nmea_core.sv

SEED ?= 1
N    ?= 2000
ERR  ?= 30

VFLAGS := --binary --timing --assert -sv -j 0 \
          -Wall -Wno-fatal -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME \
          -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-VARHIDDEN -Wno-BLKSEQ \
          --top-module tb_nmea_core

.PHONY: all lint test test_ovf regress synth wave clean

all: lint test

lint:
	$(VERILATOR) --lint-only -Wall -Wno-PINCONNECTEMPTY -sv \
	    $(RTL) $(TOP_FPGA) --top-module tang_nano_4k_top

obj_main/Vtb_nmea_core: $(RTL) $(TB)
	$(VERILATOR) $(VFLAGS) --Mdir obj_main $(RTL) $(TB)

obj_ovf/Vtb_nmea_core: $(RTL) $(TB)
	$(VERILATOR) $(VFLAGS) --Mdir obj_ovf \
	    +define+BAUD_OUT=57_600 +define+FIFO_DEPTH=128 $(RTL) $(TB)

obj_wave/Vtb_nmea_core: $(RTL) $(TB)
	$(VERILATOR) $(VFLAGS) --trace --Mdir obj_wave +define+DUMP $(RTL) $(TB)

test: obj_main/Vtb_nmea_core
	./obj_main/Vtb_nmea_core +SEED=$(SEED) +N=$(N) +ERR=$(ERR) | tee test.log
	@grep -q "TEST PASSED" test.log

test_ovf: obj_ovf/Vtb_nmea_core
	./obj_ovf/Vtb_nmea_core +SEED=$(SEED) +N=600 +ERR=$(ERR) | tee test_ovf.log
	@grep -q "TEST PASSED" test_ovf.log

regress: obj_main/Vtb_nmea_core
	@for s in 1 2 3 4 5 6 7 8; do \
	  ./obj_main/Vtb_nmea_core +SEED=$$s +N=1000 +ERR=$(ERR) > seed_$$s.log; \
	  if grep -q "TEST PASSED" seed_$$s.log; then echo "seed $$s PASS"; \
	  else echo "seed $$s FAIL"; tail -20 seed_$$s.log; exit 1; fi; \
	done

wave: obj_wave/Vtb_nmea_core
	./obj_wave/Vtb_nmea_core +SEED=$(SEED) +N=20 +ERR=$(ERR)

# Yosys reads only part of SystemVerilog, so sv2v converts it to Verilog first
synth:
	$(SV2V) $(RTL) $(TOP_FPGA) --top tang_nano_4k_top > synth_top.v
	$(YOSYS) -q -p "read_verilog synth_top.v; \
	    synth_gowin -top tang_nano_4k_top; tee -o synth_stat.txt stat"
	@cat synth_stat.txt

clean:
	rm -rf obj_* *.log *.vcd synth_stat.txt synth_top.v
