// nmea_core.sv
// NMEA0183 checksum filter / repeater:
//
//   rx -> uart_rx -> nmea_parser -> nmea_fifo -> CR/LF -> uart_tx -> tx
//                         |
//                         +-> decoded RMC / HDT / DPT values + counters
//
// Only sentences with a correct checksum are repeated on tx.
// BAUD_OUT can be higher than BAUD_IN (e.g. 4800 in, 38400 out).

module nmea_core
  import nmea_pkg::*;
#(
  parameter int CLK_HZ     = 27_000_000,
  parameter int BAUD_IN    = 4800,
  parameter int BAUD_OUT   = 4800,
  parameter int FIFO_DEPTH = 256
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        rx,
  output logic        tx,

  output logic        sent_ok,
  output logic        sent_err,
  output err_t        err_code,
  output msg_t        msg_type,
  output logic [15:0] cnt_ok,
  output logic [15:0] cnt_err,
  output logic [15:0] cnt_ovf,      // sentences lost because FIFO was full
  output logic [15:0] cnt_frame,    // UART framing errors

  output logic        rmc_upd,
  output logic [31:0] rmc_utc,
  output logic        rmc_fix,
  output logic [31:0] rmc_lat,
  output logic        rmc_lat_s,
  output logic [31:0] rmc_lon,
  output logic        rmc_lon_w,
  output logic [15:0] rmc_sog,
  output logic [15:0] rmc_cog,
  output logic        hdt_upd,
  output logic [15:0] hdt_heading,
  output logic        dpt_upd,
  output logic [15:0] dpt_depth
);

  // ---------------- receive ----------------
  logic [7:0] rx_data;
  logic       rx_valid, rx_ferr;

  uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD_IN)) u_rx (
    .clk, .rst, .rx,
    .data      (rx_data),
    .valid     (rx_valid),
    .frame_err (rx_ferr)
  );

  // ---------------- parse ----------------
  logic       fo_wr, fo_commit, fo_abort;
  logic [7:0] fo_data;

  nmea_parser u_parser (
    .clk, .rst,
    .in_valid  (rx_valid),
    .in_data   (rx_data),
    .sent_ok, .sent_err, .err_code, .msg_type,
    .fo_wr, .fo_data, .fo_commit, .fo_abort,
    .rmc_upd, .rmc_utc, .rmc_fix, .rmc_lat, .rmc_lat_s,
    .rmc_lon, .rmc_lon_w, .rmc_sog, .rmc_cog,
    .hdt_upd, .hdt_heading,
    .dpt_upd, .dpt_depth
  );

  // ---------------- store good sentences ----------------
  logic [7:0] q_data;
  logic       q_valid, q_ready, ovf;

  nmea_fifo #(.DEPTH(FIFO_DEPTH)) u_fifo (
    .clk, .rst,
    .wr       (fo_wr),
    .wr_data  (fo_data),
    .commit   (fo_commit),
    .abort    (fo_abort),
    .rd_data  (q_data),
    .rd_valid (q_valid),
    .rd_ready (q_ready),
    .overflow (ovf)
  );

  // ---------------- CR -> CR LF ----------------
  logic       send_lf;
  logic [7:0] tx_data;
  logic       tx_valid, tx_ready;

  assign tx_data  = send_lf ? CH_LF : q_data;
  assign tx_valid = send_lf | q_valid;
  assign q_ready  = tx_ready & ~send_lf;

  always_ff @(posedge clk) begin
    if (rst)
      send_lf <= 1'b0;
    else if (tx_valid && tx_ready)
      send_lf <= !send_lf && (q_data == CH_CR);
  end

  uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD_OUT)) u_tx (
    .clk, .rst,
    .data  (tx_data),
    .valid (tx_valid),
    .ready (tx_ready),
    .tx
  );

  // ---------------- statistics ----------------
  // saturating, a counter stuck at FFFF means "a lot"
  function automatic logic [15:0] inc_sat(input logic [15:0] v);
    return (v == 16'hFFFF) ? v : v + 1'b1;
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      cnt_ok    <= '0;
      cnt_err   <= '0;
      cnt_ovf   <= '0;
      cnt_frame <= '0;
    end else begin
      if (sent_ok)  cnt_ok    <= inc_sat(cnt_ok);
      if (sent_err) cnt_err   <= inc_sat(cnt_err);
      if (ovf)      cnt_ovf   <= inc_sat(cnt_ovf);
      if (rx_ferr)  cnt_frame <= inc_sat(cnt_frame);
    end
  end

endmodule
