# Open source build for Tang Nano 4K
# (sv2v + Yosys + nextpnr-himbaechel + Apicula + openFPGALoader)
#
#   make          -> nmea0183_filter.fs
#   make load     -> program the FPGA (SRAM, lost at power off)
#   make flash    -> write to the internal flash
#
# With Gowin EDA use build_gowin.tcl instead (see README).

SV2V    ?= sv2v
YOSYS   ?= yosys
NEXTPNR ?= nextpnr-himbaechel
PACK    ?= gowin_pack
LOADER  ?= openFPGALoader

DEVICE  := GW1NSR-LV4CQN48PC7/I6
FAMILY  := GW1NS-4
NAME    := nmea0183_filter

SRC := ../rtl/nmea_pkg.sv ../rtl/uart_rx.sv ../rtl/uart_tx.sv \
       ../rtl/dec_accum.sv ../rtl/nmea_parser.sv ../rtl/nmea_fifo.sv \
       ../rtl/nmea_core.sv tang_nano_4k_top.sv

.PHONY: all load flash clean

all: $(NAME).fs

build/top.v: $(SRC) | build
	$(SV2V) $(SRC) --top tang_nano_4k_top > $@

build/synth.json: build/top.v
	$(YOSYS) -q -p "read_verilog $<; synth_gowin -top tang_nano_4k_top -json $@"

build/pnr.json: build/synth.json tang_nano_4k.cst
	$(NEXTPNR) --json $< --write $@ --device $(DEVICE) \
	    --vopt family=$(FAMILY) --vopt cst=tang_nano_4k.cst --freq 27

$(NAME).fs: build/pnr.json
	$(PACK) -d $(FAMILY) -o $@ $<

build:
	mkdir -p build

load: $(NAME).fs
	$(LOADER) -b tangnano4k $<

flash: $(NAME).fs
	$(LOADER) -b tangnano4k -f $<

clean:
	rm -rf build $(NAME).fs
