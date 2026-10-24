// nmea_sva.sv
// Assertions, attached to the RTL with bind (no changes in the RTL files).

// checked on every clock, ignored during reset
`define CHK(name, prop) name: assert property (@(posedge clk) disable iff (rst) prop);

module parser_sva
  import nmea_pkg::*;
(
  input logic clk,
  input logic rst,
  input logic sent_ok,
  input logic sent_err,
  input err_t err_code,
  input logic fo_wr,
  input logic fo_commit,
  input logic fo_abort,
  input logic rmc_upd,
  input logic hdt_upd,
  input logic dpt_upd,
  input logic [6:0] len
);

  `CHK(a_ok_xor_err, !(sent_ok && sent_err))
  `CHK(a_commit_xor_abt, !(fo_commit && fo_abort))
  `CHK(a_commit_is_ok, fo_commit == sent_ok)
  `CHK(a_abort_is_err, fo_abort  == sent_err)
  `CHK(a_commit_writes, fo_commit |-> fo_wr)
  `CHK(a_err_has_code, sent_err |-> err_code != ERR_NONE)
  `CHK(a_upd_needs_ok, (rmc_upd || hdt_upd || dpt_upd) |-> sent_ok)
  `CHK(a_one_upd, $onehot0({rmc_upd, hdt_upd, dpt_upd}))
  `CHK(a_len_limit, len <= 7'(MAX_SENTENCE_LEN - 2))

endmodule

module fifo_sva #(parameter int AW = 8) (
  input logic        clk,
  input logic        rst,
  input logic [AW:0] wr_ptr,
  input logic [AW:0] commit_ptr,
  input logic [AW:0] rd_ptr,
  input logic        rd_valid,
  input logic        rd_ready
);

  // rd_ptr <= commit_ptr <= wr_ptr, and never more than DEPTH bytes stored
  `CHK(a_used_max, (wr_ptr - rd_ptr) <= (AW+1)'(1 << AW))
  `CHK(a_commit_order, (commit_ptr - rd_ptr) <= (wr_ptr - rd_ptr))
  // data that is visible stays visible until it is read
  `CHK(a_rd_stable, rd_valid && !rd_ready |=> rd_valid)

endmodule

module uart_tx_sva (
  input logic clk,
  input logic rst,
  input logic ready,
  input logic tx
);

  // when nothing is being sent the line is idle (high)
  `CHK(a_idle_high, ready |-> tx)

endmodule

bind nmea_parser parser_sva u_parser_sva (.*);
bind nmea_fifo   fifo_sva #(.AW(AW)) u_fifo_sva (.*);
bind uart_tx     uart_tx_sva u_tx_sva (.*);
