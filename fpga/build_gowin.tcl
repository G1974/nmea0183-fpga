# Build with Gowin EDA from the command line:
#   cd fpga
#   gw_sh build_gowin.tcl
# Bitstream: fpga/impl/pnr/nmea0183_filter.fs
#
# If your board has the PC6/I5 chip, change the part below.

set_device GW1NSR-LV4CQN48PC7/I6

add_file ../rtl/nmea_pkg.sv
add_file ../rtl/uart_rx.sv
add_file ../rtl/uart_tx.sv
add_file ../rtl/dec_accum.sv
add_file ../rtl/nmea_parser.sv
add_file ../rtl/nmea_fifo.sv
add_file ../rtl/nmea_core.sv
add_file tang_nano_4k_top.sv
add_file tang_nano_4k.cst
add_file tang_nano_4k.sdc

set_option -top_module tang_nano_4k_top
set_option -verilog_std sysv2017
set_option -use_done_as_gpio 1
set_option -output_base_name nmea0183_filter

run all
