// nmea_tb_pkg.sv
// Testbench side: a reference model written independently of the RTL,
// and a random sentence generator with error injection.

package nmea_tb_pkg;

  import nmea_pkg::*;

  // ------------------------------------------------------------------
  // reference model
  // ------------------------------------------------------------------

  // "3757.1234" -> fixed point with 'frac' decimals (truncate / pad)
  function automatic longint ref_fixed(string s, int frac);
    longint v = 0;
    int     got = 0;
    bit     dot = 0;
    for (int i = 0; i < s.len(); i++) begin
      if (s[i] == ".") begin
        dot = 1;
      end else if (!dot) begin
        v = v * 10 + longint'(s[i]) - 48;
      end else if (got < frac) begin
        v = v * 10 + longint'(s[i]) - 48;
        got++;
      end
    end
    for (int i = got; i < frac; i++) v = v * 10;
    return v;
  endfunction

  function automatic byte unsigned ref_csum(string body);
    byte unsigned x = 0;
    for (int i = 0; i < body.len(); i++) x ^= body[i];
    return x;
  endfunction

  // two upper case hex digits (%X is lower case in some simulators)
  function automatic string hex2(byte unsigned v);
    string d = "0123456789ABCDEF";
    int hi = int'(v[7:4]), lo = int'(v[3:0]);
    return {d.substr(hi, hi), d.substr(lo, lo)};
  endfunction

  // what the DUT must report for one sentence
  class expect_t;
    bit    good;
    err_t  err;
    msg_t  msg;
    string line;        // repeated sentence, "$...*hh" + CR LF
    // decoded values (only for good RMC / HDT / DPT)
    longint utc, lat, lon, sog, cog, hdg, dpt;
    bit     fix, lat_s, lon_w;
  endclass

  // ------------------------------------------------------------------
  // generator
  // ------------------------------------------------------------------
  typedef enum int {
    INJ_NONE,
    INJ_FLIP_CHAR,    // change one body char after checksum is computed
    INJ_BAD_CSUM,     // wrong checksum value
    INJ_BAD_HEX,      // non hex char in checksum
    INJ_NO_CSUM,      // no "*hh"
    INJ_TRUNCATE,     // stop in the middle, next '$' comes
    INJ_TOO_LONG,     // body longer than 80 chars
    INJ_NO_TERM,      // garbage instead of CR/LF
    INJ_COUNT
  } inject_t;

  class nmea_gen;

    int unsigned err_percent = 30;

    // coverage counters (simple functional coverage)
    int cov_msg [4];
    int cov_inj [INJ_COUNT];
    int cov_term[3];        // CRLF, LF, CR
    int cov_frac[6];        // number of lat decimals sent 0..5
    int cov_empty;          // RMC with empty position

    // string version of ?: (literals in ?: would become padded integers)
    function string pick(bit c, string a, string b);
      return c ? a : b;
    endfunction

    function int rnd(int lo, int hi);
      return int'($urandom_range(hi, lo));
    endfunction

    function string digits(int n);
      string s = "";
      for (int i = 0; i < n; i++) s = {s, $sformatf("%0d", rnd(0, 9))};
      return s;
    endfunction

    // number with 'ip' integer digits and 0..max_fd decimals
    function string num(int ip, int max_fd, int lo, int hi, output int fd);
      string s;
      s  = $sformatf("%0d", rnd(lo, hi));
      while (s.len() < ip) s = {"0", s};
      fd = rnd(0, max_fd);
      if (fd > 0) s = {s, ".", digits(fd)};
      return s;
    endfunction

    function string talker();
      string t [5] = '{"GP", "GN", "HE", "SD", "II"};
      return t[rnd(0, 4)];
    endfunction

    // build one sentence, fill e, return the bytes to put on the bytes
    function string next(expect_t e);
      string body, f, bytes, ck;
      int    fd, kind, inj, term;

      kind   = rnd(0, 3);
      body   = "";

      case (kind)
        // ---------------- RMC ----------------
        1: begin
          string sp, sl, so, ss, sc, st;
          int    fdl;
          bit    empty_pos = (rnd(0, 9) == 0);
          e.msg = MSG_RMC;
          st    = $sformatf("%02d%02d%02d", rnd(0, 23), rnd(0, 59), rnd(0, 59));
          if (rnd(0, 1) != 0) st = {st, ".", digits(rnd(1, 3))};
          e.fix = !empty_pos && (rnd(0, 7) != 0);
          if (empty_pos) begin
            sp = ""; so = ""; cov_empty++;
            e.lat_s = 0; e.lon_w = 0;
          end else begin
            sp = {$sformatf("%02d%02d", rnd(0, 89), rnd(0, 59)), "."};
            fdl = rnd(0, 5);
            cov_frac[fdl]++;
            sp = (fdl == 0) ? sp.substr(0, 3) : {sp, digits(fdl)};
            so = {$sformatf("%03d%02d", rnd(0, 179), rnd(0, 59)), ".", digits(rnd(1, 5))};
            e.lat_s = (rnd(0, 1) != 0);
            e.lon_w = (rnd(0, 1) != 0);
          end
          ss = num(1, 3, 0, 99, fd);
          sc = num(1, 3, 0, 359, fd);
          e.utc = ref_fixed(st, 0);
          e.lat = ref_fixed(sp, 4);
          e.lon = ref_fixed(so, 4);
          e.sog = ref_fixed(ss, 1);
          e.cog = ref_fixed(sc, 1);
          body = {talker(), "RMC,", st, ",", pick(e.fix, "A", "V"), ",",
                  sp, ",", pick(empty_pos, "", pick(e.lat_s, "S", "N")), ",",
                  so, ",", pick(empty_pos, "", pick(e.lon_w, "W", "E")), ",",
                  ss, ",", sc, ",", $sformatf("%02d%02d%02d", rnd(1,28), rnd(1,12), rnd(0,99)),
                  ",,,A"};
        end
        // ---------------- HDT ----------------
        2: begin
          e.msg = MSG_HDT;
          f     = num(1, 3, 0, 359, fd);
          e.hdg = ref_fixed(f, 1);
          body  = {talker(), "HDT,", f, ",T"};
        end
        // ---------------- DPT ----------------
        3: begin
          e.msg = MSG_DPT;
          f     = num(1, 2, 0, 999, fd);
          e.dpt = ref_fixed(f, 1);
          body  = {"SDDPT,", f, ",", num(1, 1, 0, 2, fd), ",100.0"};
        end
        // ---------------- something we do not decode ----------------
        default: begin
          e.msg = MSG_OTHER;
          case (rnd(0, 2))
            0: body = {talker(), "GGA,", digits(6), ",", digits(4), ".", digits(3), ",N,",
                       digits(5), ".", digits(3), ",E,1,08,0.9,", digits(3), ".", digits(1), ",M,,M,,"};
            1: body = {"IIMWV,", num(1, 1, 0, 359, fd), ",R,", num(1, 1, 0, 60, fd), ",N,A"};
            default: body = {"AIVDM,1,1,,A,", "15M67FC000G?ufbE`FepT@3n00Sa", ",0"};
          endcase
        end
      endcase
      cov_msg[e.msg]++;

      // ---------------- error injection ----------------
      inj = (rnd(1, 100) <= err_percent) ? rnd(1, INJ_COUNT - 1) : INJ_NONE;
      cov_inj[inj]++;

      if (inj == INJ_TOO_LONG)
        while (body.len() <= 80) body = {body, ",", digits(8)};

      ck = hex2(ref_csum(body));
      e.line = {pick(body.substr(0, 1) == "AI", "!", "$"), body, "*", ck, "\015\012"};

      case (inj)
        INJ_FLIP_CHAR: begin
          int  p = rnd(0, body.len() - 1);
          byte c;
          do c = byte'("A" + rnd(0, 25)); while (c == body[p]);
          body[p] = c;
        end
        INJ_BAD_CSUM:  ck = hex2(ref_csum(body) ^ byte'(rnd(1, 255)));
        INJ_BAD_HEX:   ck[rnd(0, 1)] = ((rnd(0, 1) != 0) ? "g" : "Z");
        default: ;
      endcase

      term = rnd(0, 2);
      cov_term[term]++;

      bytes = {e.line.substr(0, 0), body};
      case (inj)
        INJ_NO_CSUM:  bytes = {bytes, "\015\012"};
        INJ_TRUNCATE: bytes = bytes.substr(0, rnd(1, bytes.len() - 1));
        INJ_NO_TERM:  bytes = {bytes, "*", ck, "X\015\012"};
        default:      bytes = {bytes, "*", ck, pick(term == 0, "\015\012", pick(term == 1, "\012", "\015"))};
      endcase

      e.good = (inj == INJ_NONE);
      case (inj)
        INJ_NONE:      e.err = ERR_NONE;
        INJ_FLIP_CHAR,
        INJ_BAD_CSUM:  e.err = ERR_CHECKSUM;
        INJ_BAD_HEX:   e.err = ERR_BAD_HEX;
        INJ_NO_CSUM:   e.err = ERR_NO_CSUM;
        INJ_TRUNCATE:  e.err = ERR_RESTART;
        INJ_TOO_LONG:  e.err = ERR_TOO_LONG;
        default:       e.err = ERR_NO_TERM;
      endcase
      return bytes;
    endfunction

    function void report();
      $display("---- functional coverage ----");
      $display("  messages   OTHER=%0d RMC=%0d HDT=%0d DPT=%0d",
               cov_msg[0], cov_msg[1], cov_msg[2], cov_msg[3]);
      $display("  terminator CRLF=%0d LF=%0d CR=%0d", cov_term[0], cov_term[1], cov_term[2]);
      $display("  lat decimals 0..5: %0d %0d %0d %0d %0d %0d",
               cov_frac[0], cov_frac[1], cov_frac[2], cov_frac[3], cov_frac[4], cov_frac[5]);
      $display("  RMC with empty position: %0d", cov_empty);
      for (int i = 0; i < INJ_COUNT; i++) begin
        inject_t k = inject_t'(i);
        $display("  inject %-14s %0d", k.name(), cov_inj[i]);
      end
    endfunction

    // every bin must be hit at least once
    function int holes();
      int h = 0;
      foreach (cov_msg[i])  if (cov_msg[i]  == 0) h++;
      foreach (cov_term[i]) if (cov_term[i] == 0) h++;
      foreach (cov_frac[i]) if (cov_frac[i] == 0) h++;
      foreach (cov_inj[i])  if (cov_inj[i]  == 0) h++;
      if (cov_empty == 0) h++;
      return h;
    endfunction

  endclass

endpackage
