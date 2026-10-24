// tang_nano_4k_top.sv
// Board top for Sipeed Tang Nano 4K (GW1NSR-4C).
//
//   pin 45  clk_27m   27 MHz oscillator
//   pin 15  key_n     button, reset (active low)
//   pin 40  nmea_rx   NMEA in  (3.3V TTL, from an RS-422 receiver!)
//   pin 39  nmea_tx   NMEA out (only good sentences)
//   pin 10  led       short flash = good sentence, long = error
//
// Do NOT connect the NMEA RS-422 lines directly to the FPGA.
// Use an RS-422 / RS-485 receiver (e.g. MAX3485, SN65HVD) or an
// opto-isolator in front of pin 40.

module tang_nano_4k_top #(
  parameter int  CLK_HZ   = 27_000_000,
  parameter int  BAUD_IN  = 4800,
  parameter int  BAUD_OUT = 4800,
  parameter bit  LED_LOW  = 1'b1        // LED is on when the pin is 0
) (
  input  logic clk_27m,
  input  logic key_n,
  input  logic nmea_rx,
  output logic nmea_tx,
  output logic led
);

  // ---------------- reset ----------------
  // power-on reset for a few clocks, then follow the button
  logic [3:0] por_cnt = '0;
  logic       key_meta = 1'b1, key_s = 1'b1;
  logic       rst = 1'b1;

  always_ff @(posedge clk_27m) begin
    key_meta <= key_n;
    key_s    <= key_meta;
    if (por_cnt != 4'hF) por_cnt <= por_cnt + 1'b1;
    rst <= (por_cnt != 4'hF) || !key_s;
  end

  // ---------------- core ----------------
  logic sent_ok, sent_err;

  nmea_core #(
    .CLK_HZ   (CLK_HZ),
    .BAUD_IN  (BAUD_IN),
    .BAUD_OUT (BAUD_OUT)
  ) u_core (
    .clk      (clk_27m),
    .rst      (rst),
    .rx       (nmea_rx),
    .tx       (nmea_tx),
    .sent_ok  (sent_ok),
    .sent_err (sent_err),
    // the decoded values are not used on this board yet,
    // they are ready for an HDMI or SPI display later
    .err_code (), .msg_type (),
    .cnt_ok (), .cnt_err (), .cnt_ovf (), .cnt_frame (),
    .rmc_upd (), .rmc_utc (), .rmc_fix (), .rmc_lat (), .rmc_lat_s (),
    .rmc_lon (), .rmc_lon_w (), .rmc_sog (), .rmc_cog (),
    .hdt_upd (), .hdt_heading (),
    .dpt_upd (), .dpt_depth ()
  );

  // ---------------- LED ----------------
  localparam int T_OK  = CLK_HZ / 33;   // ~30 ms
  localparam int T_ERR = CLK_HZ / 3;    // ~330 ms
  localparam int LW    = $clog2(T_ERR + 1);

  logic [LW-1:0] led_cnt;

  always_ff @(posedge clk_27m) begin
    if (rst)
      led_cnt <= '0;
    else if (sent_err)
      led_cnt <= LW'(T_ERR);
    else if (sent_ok && led_cnt < LW'(T_OK))
      led_cnt <= LW'(T_OK);
    else if (led_cnt != 0)
      led_cnt <= led_cnt - 1'b1;
  end

  assign led = (led_cnt != 0) ^ LED_LOW;

endmodule
