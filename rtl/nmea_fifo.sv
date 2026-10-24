// nmea_fifo.sv
// Byte FIFO with commit / abort, so that only complete, good sentences
// reach the output.
//
//   rd_ptr        commit_ptr        wr_ptr
//     |  readable    |  speculative   |  free
//   --+==============+----------------+--------
//
// wr     : write a byte at wr_ptr (not visible to the reader yet)
// commit : everything up to wr_ptr becomes readable (a wr in the same
//          cycle is included)
// abort  : wr_ptr goes back to commit_ptr (a wr in the same cycle is the
//          first byte of the next sentence and is kept)
// If the FIFO fills up in the middle of a sentence the whole sentence is
// dropped at commit time and overflow pulses.

module nmea_fifo #(
  parameter int DEPTH = 256       // power of 2
) (
  input  logic       clk,
  input  logic       rst,

  input  logic       wr,
  input  logic [7:0] wr_data,
  input  logic       commit,
  input  logic       abort,

  output logic [7:0] rd_data,
  output logic       rd_valid,
  input  logic       rd_ready,

  output logic       overflow
);

  localparam int AW = $clog2(DEPTH);

  if (DEPTH != (1 << AW)) begin : g_check
    $error("nmea_fifo: DEPTH must be a power of 2");
  end

  logic [7:0]  mem [DEPTH];
  logic [AW:0] wr_ptr, commit_ptr, rd_ptr;   // extra MSB for full/empty
  logic        lost;                         // a byte of this sentence was lost

  // after an abort we write from commit_ptr again
  logic [AW:0] wa;
  logic        full;
  assign wa   = abort ? commit_ptr : wr_ptr;
  assign full = ((wa - rd_ptr) == (AW+1)'(DEPTH));

  assign rd_valid = (rd_ptr != commit_ptr);
  assign rd_data  = mem[rd_ptr[AW-1:0]];

  always_ff @(posedge clk) begin
    if (wr && !full)
      mem[wa[AW-1:0]] <= wr_data;
  end

  always_ff @(posedge clk) begin
    overflow <= 1'b0;

    if (rst) begin
      wr_ptr     <= '0;
      commit_ptr <= '0;
      rd_ptr     <= '0;
      lost       <= 1'b0;
    end else begin
      if (rd_valid && rd_ready)
        rd_ptr <= rd_ptr + 1'b1;

      if (commit) begin
        if (lost || (wr && full)) begin
          wr_ptr   <= commit_ptr;           // drop the broken sentence
          overflow <= 1'b1;
        end else begin
          wr_ptr     <= wa + (AW+1)'(wr);
          commit_ptr <= wa + (AW+1)'(wr);
        end
        lost <= 1'b0;
      end else begin
        if (wr && !full) wr_ptr <= wa + 1'b1;
        else             wr_ptr <= wa;
        if (abort)          lost <= wr && full;
        else if (wr && full) lost <= 1'b1;
      end
    end
  end

endmodule
