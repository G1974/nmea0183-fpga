// dec_accum.sv
// Turns an ASCII decimal field like "3757.1234" into a fixed point integer.
// frac_digits says how many decimals we keep:
//   "123.4"  frac=1 -> 1234
//   "123.45" frac=1 -> 1234   (extra decimals are dropped, not rounded)
//   "123"    frac=2 -> 12300  (missing decimals are padded)
// Bytes arrive thousands of clocks apart, so a multiply per char is fine.

module dec_accum
  import nmea_pkg::*;
#(
  parameter int W        = 32,
  parameter int MAX_FRAC = 4
) (
  input  logic         clk,
  input  logic         rst,
  input  logic         clear,        // start of a new field
  input  logic [2:0]   frac_digits,  // sampled on clear
  input  logic         push,         // ch is part of the field
  input  logic [7:0]   ch,
  output logic [W-1:0] value,        // scaled result, valid any time
  output logic         has_digits,   // field was not empty
  output logic         bad_char      // something other than 0-9 or one '.'
);

  logic [W-1:0] acc;
  logic [2:0]   frac_want;
  logic [2:0]   frac_got;
  logic         seen_dot;

  logic [3:0] digit;
  assign digit = 4'(ch - 8'h30);

  always_ff @(posedge clk) begin
    if (rst || clear) begin
      acc        <= '0;
      frac_got   <= '0;
      seen_dot   <= 1'b0;
      has_digits <= 1'b0;
      bad_char   <= 1'b0;
      if (clear) frac_want <= frac_digits;
      if (rst)   frac_want <= '0;
    end else if (push) begin
      if (is_digit(ch)) begin
        has_digits <= 1'b1;
        if (!seen_dot) begin
          acc <= acc * 10 + W'(digit);
        end else if (frac_got < frac_want) begin
          acc      <= acc * 10 + W'(digit);
          frac_got <= frac_got + 1'b1;
        end
      end else if (ch == CH_DOT && !seen_dot) begin
        seen_dot <= 1'b1;
      end else begin
        bad_char <= 1'b1;
      end
    end
  end

  // pad the decimals that did not arrive
  always_comb begin
    case (frac_want - frac_got)
      3'd0:    value = acc;
      3'd1:    value = acc * 10;
      3'd2:    value = acc * 100;
      3'd3:    value = acc * 1000;
      default: value = acc * 10000;
    endcase
  end

  if (MAX_FRAC > 4) begin : g_check
    $error("dec_accum: MAX_FRAC > 4 is not supported");
  end

endmodule
