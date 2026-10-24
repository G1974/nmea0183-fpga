// tb_nmea_core.sv
// Random test for nmea_core:
//   - driver   : sends generated sentences on rx, with baud error +-2 %
//   - monitor  : receives the repeated sentences on tx
//   - scoreboard checks
//       * every repeated line is a good sentence, in the same order
//       * the decoded RMC / HDT / DPT values match the reference model
//       * ok / error counters and error codes
//
// Plusargs:  +SEED=n  +N=number_of_sentences  +ERR=percent_bad

`ifndef CLK_HZ
  `define CLK_HZ 11_059_200
`endif
`ifndef BAUD_IN
  `define BAUD_IN 115_200
`endif
`ifndef BAUD_OUT
  `define BAUD_OUT 230_400
`endif
`ifndef FIFO_DEPTH
  `define FIFO_DEPTH 256
`endif

module tb_nmea_core;

  import nmea_pkg::*;
  import nmea_tb_pkg::*;

  localparam int CPB_IN  = `CLK_HZ / `BAUD_IN;
  localparam int CPB_OUT = `CLK_HZ / `BAUD_OUT;
  // in the overflow test the output is slower, sentences may be dropped
  localparam bit DROPS_OK = (`BAUD_OUT < `BAUD_IN) || (`FIFO_DEPTH < 128);

  logic clk = 0;
  logic rst = 1;
  logic rx  = 1;
  logic tx;

  always #5 clk = ~clk;

  logic        sent_ok, sent_err;
  err_t        err_code;
  msg_t        msg_type;
  logic [15:0] cnt_ok, cnt_err, cnt_ovf, cnt_frame;
  logic        rmc_upd, rmc_fix, rmc_lat_s, rmc_lon_w, hdt_upd, dpt_upd;
  logic [31:0] rmc_utc, rmc_lat, rmc_lon;
  logic [15:0] rmc_sog, rmc_cog, hdt_heading, dpt_depth;

  nmea_core #(
    .CLK_HZ     (`CLK_HZ),
    .BAUD_IN    (`BAUD_IN),
    .BAUD_OUT   (`BAUD_OUT),
    .FIFO_DEPTH (`FIFO_DEPTH)
  ) dut (.*);

  // ------------------------------------------------------------------
  // bookkeeping
  // ------------------------------------------------------------------
  expect_t exp_status [$];   // one per sentence, in order
  expect_t exp_decode [$];   // good RMC/HDT/DPT only
  string   exp_lines  [$];   // good sentences, what tx must repeat

  int errors   = 0;
  int n_good   = 0;
  int n_bad    = 0;
  int n_lines  = 0;
  int n_dropped = 0;

  function void fail(string msg);
    errors++;
    $display("[%0t] ERROR: %s", $time, msg);
  endfunction

  // ------------------------------------------------------------------
  // driver
  // ------------------------------------------------------------------
  task automatic send_byte(byte unsigned b, int cpb);
    rx = 0;                      repeat (cpb) @(posedge clk);
    for (int i = 0; i < 8; i++) begin
      rx = b[i];                 repeat (cpb) @(posedge clk);
    end
    rx = 1;                      repeat (cpb) @(posedge clk);
  endtask

  task automatic send_string(string s, int cpb);
    for (int i = 0; i < s.len(); i++) send_byte(s[i], cpb);
  endtask

  // ------------------------------------------------------------------
  // status scoreboard: one ok or err pulse per sentence
  // ------------------------------------------------------------------
  always @(posedge clk) begin
    if (!rst && (sent_ok || sent_err)) begin
      if (exp_status.size() == 0) begin
        fail("status pulse with no sentence pending");
      end else begin
        automatic expect_t e = exp_status.pop_front();
        if (sent_ok !== e.good)
          fail($sformatf("got %s, expected %s for %s", sent_ok ? "OK" : "ERR",
                         e.good ? "OK" : "ERR", e.line));
        else if (sent_err && err_code !== e.err)
          fail($sformatf("error code %s, expected %s", err_code.name(), e.err.name()));
        else if (sent_ok && msg_type !== e.msg)
          fail($sformatf("msg type %s, expected %s", msg_type.name(), e.msg.name()));
      end
    end
  end

  // ------------------------------------------------------------------
  // decode scoreboard
  // ------------------------------------------------------------------
  always @(posedge clk) begin
    if (!rst && (rmc_upd || hdt_upd || dpt_upd)) begin
      if (exp_decode.size() == 0) begin
        fail("decode update with nothing expected");
      end else begin
        automatic expect_t e = exp_decode.pop_front();
        if (rmc_upd) begin
          if (e.msg != MSG_RMC)                 fail("unexpected RMC update");
          else if (rmc_utc   != 32'(e.utc))     fail($sformatf("RMC utc %0d != %0d", rmc_utc, e.utc));
          else if (rmc_fix   != e.fix)          fail("RMC fix");
          else if (rmc_lat   != 32'(e.lat))     fail($sformatf("RMC lat %0d != %0d  %s", rmc_lat, e.lat, e.line));
          else if (rmc_lat_s != e.lat_s)        fail("RMC N/S");
          else if (rmc_lon   != 32'(e.lon))     fail($sformatf("RMC lon %0d != %0d  %s", rmc_lon, e.lon, e.line));
          else if (rmc_lon_w != e.lon_w)        fail("RMC E/W");
          else if (rmc_sog   != 16'(e.sog))     fail($sformatf("RMC sog %0d != %0d", rmc_sog, e.sog));
          else if (rmc_cog   != 16'(e.cog))     fail($sformatf("RMC cog %0d != %0d", rmc_cog, e.cog));
        end
        if (hdt_upd) begin
          if (e.msg != MSG_HDT)                 fail("unexpected HDT update");
          else if (hdt_heading != 16'(e.hdg))   fail($sformatf("HDT %0d != %0d  %s", hdt_heading, e.hdg, e.line));
        end
        if (dpt_upd) begin
          if (e.msg != MSG_DPT)                 fail("unexpected DPT update");
          else if (dpt_depth != 16'(e.dpt))     fail($sformatf("DPT %0d != %0d  %s", dpt_depth, e.dpt, e.line));
        end
      end
    end
  end

  // ------------------------------------------------------------------
  // monitor on tx + line scoreboard
  // ------------------------------------------------------------------
  string cur_line = "";

  task automatic check_line(string l);
    n_lines++;
    // in the overflow test some good sentences may be missing
    while (exp_lines.size() > 0 && exp_lines[0] != l && DROPS_OK) begin
      void'(exp_lines.pop_front());
      n_dropped++;
    end
    if (exp_lines.size() == 0)
      fail($sformatf("tx sent a line that was not expected: %s", l));
    else if (exp_lines[0] != l)
      fail($sformatf("tx line mismatch\n   got: %s\n   exp: %s", l, exp_lines[0]));
    else
      void'(exp_lines.pop_front());
  endtask

  initial begin
    byte unsigned b;
    @(negedge rst);
    forever begin
      @(negedge tx);
      repeat (CPB_OUT / 2) @(posedge clk);
      if (tx !== 0) fail("tx start bit glitch");
      for (int i = 0; i < 8; i++) begin
        repeat (CPB_OUT) @(posedge clk);
        b[i] = tx;
      end
      repeat (CPB_OUT) @(posedge clk);
      if (tx !== 1) fail("tx stop bit missing");
      cur_line = {cur_line, $sformatf("%c", b)};
      if (b == 8'h0A) begin
        check_line(cur_line);
        cur_line = "";
      end
    end
  end

  // ------------------------------------------------------------------
  // main
  // ------------------------------------------------------------------
  initial begin
    nmea_gen gen = new();
    int      seed = 1, n = 2000, err = 30;
    int      cpb;
    string   w;

    void'($value$plusargs("SEED=%d", seed));
    void'($value$plusargs("N=%d", n));
    void'($value$plusargs("ERR=%d", err));
    process::self().srandom(seed);
    gen.err_percent = err;

    $display("tb_nmea_core: seed=%0d n=%0d err=%0d%%  CLK=%0d in=%0d out=%0d fifo=%0d",
             seed, n, err, `CLK_HZ, `BAUD_IN, `BAUD_OUT, `FIFO_DEPTH);

    repeat (10) @(posedge clk);
    rst = 0;
    repeat (10) @(posedge clk);

    // ---- directed: the classic example from the NMEA docs ----
    begin
      automatic expect_t e = new();
      e.good = 1; e.err = ERR_NONE; e.msg = MSG_RMC;
      e.utc = 123519; e.fix = 1;
      e.lat = 48070380;  e.lat_s = 0;
      e.lon = 11310000;  e.lon_w = 0;
      e.sog = 224; e.cog = 844;
      e.line = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\015\012";
      exp_status.push_back(e);
      exp_decode.push_back(e);
      exp_lines.push_back(e.line);
      n_good++;
      send_string(e.line, CPB_IN);
    end

    // ---- random ----
    for (int i = 0; i < n; i++) begin
      automatic expect_t e = new();
      w = gen.next(e);

      // a truncated sentence is only an error if another '$' follows it,
      // that is always true here because a good one is sent at the end
      exp_status.push_back(e);
      if (e.good) begin
        n_good++;
        exp_lines.push_back(e.line);
        if (e.msg != MSG_OTHER) exp_decode.push_back(e);
      end else begin
        n_bad++;
      end

      // baud rate error up to +-2 %
      cpb = (CPB_IN * (98 + $urandom_range(4, 0))) / 100;
      send_string(w, cpb);

      // idle gap, sometimes with noise (not after a truncated sentence)
      if (e.err != ERR_RESTART) begin
        repeat ($urandom_range(3 * CPB_IN, 0)) @(posedge clk);
        if ($urandom_range(9, 0) == 0) send_string("noise 123 ,*", CPB_IN);
      end
    end

    // one last good sentence closes any truncated one
    begin
      automatic expect_t e = new();
      e.good = 1; e.err = ERR_NONE; e.msg = MSG_HDT; e.hdg = 900;
      e.line = {"$HEHDT,90.0,T*", hex2(ref_csum("HEHDT,90.0,T")), "\015\012"};
      exp_status.push_back(e);
      exp_decode.push_back(e);
      exp_lines.push_back(e.line);
      n_good++;
      send_string(e.line, CPB_IN);
    end

    // wait until everything has been sent out
    while (dut.u_fifo.rd_valid || !dut.u_tx.ready || dut.send_lf)
      @(posedge clk);
    repeat (20 * CPB_OUT) @(posedge clk);

    // ---- final checks ----
    if (exp_status.size() != 0) fail($sformatf("%0d sentences got no status", exp_status.size()));
    if (exp_decode.size() != 0) fail($sformatf("%0d decode updates missing", exp_decode.size()));
    n_dropped += exp_lines.size();
    if (cnt_ok  != 16'(n_good)) fail($sformatf("cnt_ok %0d != %0d", cnt_ok, n_good));
    if (cnt_err != 16'(n_bad))  fail($sformatf("cnt_err %0d != %0d", cnt_err, n_bad));
    if (cnt_frame != 0)         fail($sformatf("cnt_frame %0d", cnt_frame));
    if (n_dropped != int'(cnt_ovf))
      fail($sformatf("%0d lines missing on tx but cnt_ovf = %0d", n_dropped, cnt_ovf));
    if (!DROPS_OK && n_dropped != 0)
      fail("sentences were dropped but the FIFO should be big enough");
    if (DROPS_OK && cnt_ovf == 0)
      fail("overflow test did not overflow");

    gen.report();
    $display("---- result ----");
    $display("  good %0d  bad %0d  repeated %0d  dropped(ovf) %0d",
             n_good, n_bad, n_lines, n_dropped);
    if (gen.holes() != 0 && n >= 500) fail($sformatf("%0d coverage bins not hit", gen.holes()));

    if (errors == 0) $display("TEST PASSED");
    else             $display("TEST FAILED (%0d errors)", errors);
    $finish;
  end

`ifdef DUMP
  initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0, tb_nmea_core);
  end
`endif

  // watchdog
  initial begin
    #(64'd2_000_000_000_000);
    $display("TEST FAILED (timeout)");
    $finish;
  end

endmodule
