// nmea_pkg.sv
// Shared constants for the NMEA0183 receiver.

package nmea_pkg;

  // NMEA0183 limit: 82 chars including '$' and <CR><LF>
  localparam int MAX_SENTENCE_LEN = 82;

  // ASCII
  localparam logic [7:0] CH_DOLLAR = 8'h24;  // '$'
  localparam logic [7:0] CH_BANG   = 8'h21;  // '!'  (AIS encapsulation)
  localparam logic [7:0] CH_STAR   = 8'h2A;  // '*'
  localparam logic [7:0] CH_COMMA  = 8'h2C;  // ','
  localparam logic [7:0] CH_DOT    = 8'h2E;  // '.'
  localparam logic [7:0] CH_CR     = 8'h0D;
  localparam logic [7:0] CH_LF     = 8'h0A;

  // Why a sentence was rejected
  typedef enum logic [2:0] {
    ERR_NONE     = 3'd0,
    ERR_CHECKSUM = 3'd1,  // checksum mismatch
    ERR_BAD_HEX  = 3'd2,  // char after '*' is not a hex digit
    ERR_TOO_LONG = 3'd3,  // more than 82 chars
    ERR_NO_TERM  = 3'd4,  // something other than CR/LF after checksum
    ERR_RESTART  = 3'd5,  // new '$' before the previous sentence ended
    ERR_NO_CSUM  = 3'd6   // line ended without "*hh"
  } err_t;

  // Sentence types we decode (talker ID is ignored)
  typedef enum logic [1:0] {
    MSG_OTHER = 2'd0,
    MSG_RMC   = 2'd1,
    MSG_HDT   = 2'd2,
    MSG_DPT   = 2'd3
  } msg_t;

  function automatic logic is_digit(input logic [7:0] c);
    is_digit = (c >= 8'h30) && (c <= 8'h39);            // '0'..'9'
  endfunction

  function automatic logic is_hex(input logic [7:0] c);
    is_hex = is_digit(c) || ((c >= 8'h41) && (c <= 8'h46));  // 'A'..'F'
  endfunction

  // Only valid when is_hex(c). Lower case is not allowed by the standard.
  // '0'..'9' = 0x30..0x39, 'A'..'F' = 0x41..0x46
  function automatic logic [3:0] hex_val(input logic [7:0] c);
    hex_val = is_digit(c) ? c[3:0] : (c[3:0] + 4'd9);
  endfunction

endpackage
