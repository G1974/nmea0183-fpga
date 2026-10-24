// nmea_parser.sv
// Byte-by-byte NMEA0183 parser.
//
//   $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A<CR><LF>
//   ^      ^                                                          ^^^
//   start  fields (field 0 = address)                      checksum = XOR of
//                                                          everything between
//                                                          '$' and '*'
//
// Does three things in parallel:
//   1. checks framing + checksum and reports ok / error per sentence
//   2. decodes a few fields (RMC, HDT, DPT) into shadow registers and copies
//      them to the outputs only when the checksum is correct
//   3. sends every byte of the sentence to the FIFO port (wr/commit/abort),
//      so a FIFO can keep only the good sentences (see nmea_fifo.sv)
//
// All outputs are registered (1 clock after the input byte).

module nmea_parser
  import nmea_pkg::*;
(
  input  logic        clk,
  input  logic        rst,

  // byte stream from uart_rx
  input  logic        in_valid,
  input  logic [7:0]  in_data,

  // per sentence status
  output logic        sent_ok,     // pulse
  output logic        sent_err,    // pulse
  output err_t        err_code,    // valid with sent_err
  output msg_t        msg_type,    // valid with sent_ok

  // FIFO port
  output logic        fo_wr,
  output logic [7:0]  fo_data,
  output logic        fo_commit,   // keep the bytes written since start
  output logic        fo_abort,    // throw them away

  // RMC - recommended minimum GNSS data
  output logic        rmc_upd,
  output logic [31:0] rmc_utc,     // hhmmss as decimal, e.g. 123519
  output logic        rmc_fix,     // status 'A'
  output logic [31:0] rmc_lat,     // ddmm.mmmm * 10^4
  output logic        rmc_lat_s,   // 1 = South
  output logic [31:0] rmc_lon,     // dddmm.mmmm * 10^4
  output logic        rmc_lon_w,   // 1 = West
  output logic [15:0] rmc_sog,     // knots * 10
  output logic [15:0] rmc_cog,     // degrees * 10

  // HDT - true heading (gyro)
  output logic        hdt_upd,
  output logic [15:0] hdt_heading, // degrees * 10

  // DPT - depth (echo sounder)
  output logic        dpt_upd,
  output logic [15:0] dpt_depth    // metres * 10
);

  // sentence length without <CR><LF>
  localparam int MAX_LEN = MAX_SENTENCE_LEN - 2;

  typedef enum logic [2:0] {IDLE, BODY, CK_HI, CK_LO, TERM} state_t;
  state_t state;

  logic [7:0] csum;
  logic [3:0] ck_hi;
  logic [6:0] len;
  logic [4:0] field;       // field index, 0 = address
  logic [2:0] fpos;        // char position inside field (saturates)
  logic [7:0] fch;         // first char of current field
  logic [7:0] typ [3];     // address chars 2..4, e.g. "RMC"
  msg_t       msg;

  // ------------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------------
  logic [7:0] c;
  logic       is_start, is_end;
  assign c        = in_data;
  assign is_start = (c == CH_DOLLAR) || (c == CH_BANG);
  assign is_end   = (c == CH_CR) || (c == CH_LF);

  function automatic msg_t decode_type(input logic [7:0] t0, t1, t2);
    if      ({t0, t1, t2} == "RMC") return MSG_RMC;
    else if ({t0, t1, t2} == "HDT") return MSG_HDT;
    else if ({t0, t1, t2} == "DPT") return MSG_DPT;
    else                            return MSG_OTHER;
  endfunction

  // how many decimals to keep for each numeric field
  function automatic logic [2:0] frac_for(input msg_t m, input logic [4:0] f);
    case (m)
      MSG_RMC: case (f)
                 5'd3, 5'd5: return 3'd4;   // lat / lon
                 5'd7, 5'd8: return 3'd1;   // sog / cog
                 default:    return 3'd0;   // time: integer part only
               endcase
      MSG_HDT, MSG_DPT: return 3'd1;
      default:          return 3'd0;
    endcase
  endfunction

  // msg is decided at the end of field 0, use the fresh value in that cycle
  msg_t msg_now;
  assign msg_now = (field == 0) ? decode_type(typ[0], typ[1], typ[2]) : msg;

  // ------------------------------------------------------------------
  // decimal field accumulator
  // ------------------------------------------------------------------
  logic        in_body_char;   // ordinary char inside the body
  logic        field_sep;      // ',' or '*' while in body
  logic        acc_clear, acc_push;
  logic [31:0] acc_val;

  assign in_body_char = in_valid && (state == BODY) && !is_start && !is_end
                        && (c != CH_STAR) && (c != CH_COMMA);
  assign field_sep    = in_valid && (state == BODY)
                        && ((c == CH_COMMA) || (c == CH_STAR));

  assign acc_clear = field_sep && (c == CH_COMMA);
  assign acc_push  = in_body_char && (field != 0);

  dec_accum #(.W(32)) u_acc (
    .clk         (clk),
    .rst         (rst),
    .clear       (acc_clear),
    .frac_digits (frac_for(msg_now, field + 5'd1)),
    .push        (acc_push),
    .ch          (c),
    .value       (acc_val),
    .has_digits  (),
    .bad_char    ()
  );

  // ------------------------------------------------------------------
  // shadow registers, filled while the sentence comes in
  // ------------------------------------------------------------------
  logic [31:0] sh_utc, sh_lat, sh_lon;
  logic [15:0] sh_sog, sh_cog, sh_hdg, sh_dpt;
  logic        sh_fix, sh_lat_s, sh_lon_w;

  always_ff @(posedge clk) begin
    if (rst || (in_valid && is_start)) begin
      sh_utc <= '0; sh_lat <= '0; sh_lon <= '0;
      sh_sog <= '0; sh_cog <= '0; sh_hdg <= '0; sh_dpt <= '0;
      sh_fix <= 1'b0; sh_lat_s <= 1'b0; sh_lon_w <= 1'b0;
    end else if (field_sep && field != 0) begin
      // field just ended: acc_val / fch hold its content
      case (msg)
        MSG_RMC: case (field)
          5'd1: sh_utc   <= acc_val;
          5'd2: sh_fix   <= (fch == "A");
          5'd3: sh_lat   <= acc_val;
          5'd4: sh_lat_s <= (fch == "S");
          5'd5: sh_lon   <= acc_val;
          5'd6: sh_lon_w <= (fch == "W");
          5'd7: sh_sog   <= acc_val[15:0];
          5'd8: sh_cog   <= acc_val[15:0];
          default: ;
        endcase
        MSG_HDT: if (field == 5'd1) sh_hdg <= acc_val[15:0];
        MSG_DPT: if (field == 5'd1) sh_dpt <= acc_val[15:0];
        default: ;
      endcase
    end
  end

  // ------------------------------------------------------------------
  // main FSM
  // ------------------------------------------------------------------
  always_ff @(posedge clk) begin : fsm
    err_t fail;   // set below when the sentence has to be dropped
    fail = ERR_NONE;

    // pulses
    sent_ok   <= 1'b0;
    sent_err  <= 1'b0;
    fo_wr     <= 1'b0;
    fo_commit <= 1'b0;
    fo_abort  <= 1'b0;
    rmc_upd   <= 1'b0;
    hdt_upd   <= 1'b0;
    dpt_upd   <= 1'b0;

    if (rst) begin
      state    <= IDLE;
      csum     <= '0;
      ck_hi    <= '0;
      len      <= '0;
      field    <= '0;
      fpos     <= '0;
      fch      <= '0;
      typ      <= '{default: '0};
      msg      <= MSG_OTHER;
      err_code <= ERR_NONE;
      msg_type <= MSG_OTHER;
      fo_data  <= '0;
      rmc_utc <= '0; rmc_fix <= 1'b0; rmc_lat <= '0; rmc_lat_s <= 1'b0;
      rmc_lon <= '0; rmc_lon_w <= 1'b0; rmc_sog <= '0; rmc_cog <= '0;
      hdt_heading <= '0;
      dpt_depth   <= '0;
    end else if (in_valid) begin
      fo_data <= c;

      if (is_start) begin
        // '$' always starts a new sentence, even in the middle of one
        if (state != IDLE) begin
          sent_err <= 1'b1;
          err_code <= ERR_RESTART;
          fo_abort <= 1'b1;
        end
        state <= BODY;
        csum  <= '0;
        len   <= 7'd1;
        field <= '0;
        fpos  <= '0;
        typ   <= '{default: '0};
        msg   <= MSG_OTHER;
        fo_wr <= 1'b1;

      end else begin
        case (state)
          IDLE: ;  // ignore everything until '$'

          BODY: begin
            if (is_end) begin
              fail = ERR_NO_CSUM;
            end else if (len == 7'(MAX_LEN)) begin
              fail = ERR_TOO_LONG;
            end else begin
              len   <= len + 1'b1;
              fo_wr <= 1'b1;
              if (c == CH_STAR) begin
                state <= CK_HI;
                if (field == 0) msg <= msg_now;
              end else begin
                csum <= csum ^ c;
                if (c == CH_COMMA) begin
                  if (field == 0) msg <= msg_now;
                  if (field != 5'd31) field <= field + 1'b1;
                  fpos <= '0;
                  fch  <= '0;
                end else begin
                  if (fpos == 0) fch <= c;
                  if (fpos != 3'd7) fpos <= fpos + 1'b1;
                  // address field: "GPRMC" -> keep chars 2,3,4
                  if (field == 0 && fpos >= 3'd2 && fpos <= 3'd4)
                    typ[2'(fpos - 3'd2)] <= c;
                end
              end
            end
          end

          CK_HI: begin
            if (!is_hex(c)) begin
              fail = ERR_BAD_HEX;
            end else begin
              ck_hi <= hex_val(c);
              len   <= len + 1'b1;
              fo_wr <= 1'b1;
              state <= CK_LO;
            end
          end

          CK_LO: begin
            if (!is_hex(c)) begin
              fail = ERR_BAD_HEX;
            end else if ({ck_hi, hex_val(c)} != csum) begin
              fail = ERR_CHECKSUM;
            end else begin
              fo_wr <= 1'b1;
              state <= TERM;
            end
          end

          // checksum was right, we only need the line end
          TERM: begin
            if (is_end) begin
              state     <= IDLE;
              fo_wr     <= 1'b1;
              fo_data   <= CH_CR;   // store one terminator, the TX adds LF
              fo_commit <= 1'b1;
              sent_ok   <= 1'b1;
              msg_type  <= msg;
              case (msg)
                MSG_RMC: begin
                  rmc_upd   <= 1'b1;
                  rmc_utc   <= sh_utc;
                  rmc_fix   <= sh_fix;
                  rmc_lat   <= sh_lat;
                  rmc_lat_s <= sh_lat_s;
                  rmc_lon   <= sh_lon;
                  rmc_lon_w <= sh_lon_w;
                  rmc_sog   <= sh_sog;
                  rmc_cog   <= sh_cog;
                end
                MSG_HDT: begin
                  hdt_upd     <= 1'b1;
                  hdt_heading <= sh_hdg;
                end
                MSG_DPT: begin
                  dpt_upd   <= 1'b1;
                  dpt_depth <= sh_dpt;
                end
                default: ;
              endcase
            end else begin
              fail = ERR_NO_TERM;
            end
          end

          default: state <= IDLE;
        endcase

        if (fail != ERR_NONE) begin
          state    <= IDLE;
          sent_err <= 1'b1;
          err_code <= fail;
          fo_abort <= 1'b1;
          fo_wr    <= 1'b0;
        end
      end
    end
  end

endmodule
