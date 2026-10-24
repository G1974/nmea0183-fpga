// uart_rx.sv
// 8N1 UART receiver. Samples each bit in the middle of the bit period.
// The input goes through a 2-FF synchronizer first, it comes from outside.

module uart_rx #(
  parameter int CLK_HZ = 27_000_000,
  parameter int BAUD   = 4800
) (
  input  logic       clk,
  input  logic       rst,
  input  logic       rx,
  output logic [7:0] data,
  output logic       valid,      // 1 clock pulse, data is good
  output logic       frame_err   // 1 clock pulse, stop bit was 0
);

  localparam int CLKS_PER_BIT = CLK_HZ / BAUD;
  localparam int CW           = $clog2(CLKS_PER_BIT);

  // elaboration time check
  if (CLKS_PER_BIT < 8) begin : g_check
    $error("uart_rx: CLK_HZ too low for BAUD");
  end

  // synchronizer, idle line is high
  logic rx_meta, rx_s;
  always_ff @(posedge clk) begin
    if (rst) {rx_s, rx_meta} <= 2'b11;
    else     {rx_s, rx_meta} <= {rx_meta, rx};
  end

  typedef enum logic [1:0] {IDLE, START, DATA, STOP} state_t;
  state_t state;

  logic [CW-1:0] cnt;
  logic [2:0]    bit_idx;
  logic [7:0]    shreg;

  always_ff @(posedge clk) begin
    valid     <= 1'b0;
    frame_err <= 1'b0;

    if (rst) begin
      state   <= IDLE;
      cnt     <= '0;
      bit_idx <= '0;
      shreg   <= '0;
      data    <= '0;
    end else begin
      case (state)
        IDLE: begin
          cnt <= '0;
          if (!rx_s) state <= START;
        end

        // wait half a bit and check the start bit again (glitch filter)
        START: begin
          if (cnt == CW'(CLKS_PER_BIT/2 - 1)) begin
            cnt     <= '0;
            bit_idx <= '0;
            state   <= rx_s ? IDLE : DATA;
          end else begin
            cnt <= cnt + 1'b1;
          end
        end

        DATA: begin
          if (cnt == CW'(CLKS_PER_BIT - 1)) begin
            cnt   <= '0;
            shreg <= {rx_s, shreg[7:1]};   // LSB first
            if (bit_idx == 3'd7) state <= STOP;
            bit_idx <= bit_idx + 1'b1;
          end else begin
            cnt <= cnt + 1'b1;
          end
        end

        STOP: begin
          if (cnt == CW'(CLKS_PER_BIT - 1)) begin
            cnt   <= '0;
            state <= IDLE;
            if (rx_s) begin
              data  <= shreg;
              valid <= 1'b1;
            end else begin
              frame_err <= 1'b1;
            end
          end else begin
            cnt <= cnt + 1'b1;
          end
        end

        default: state <= IDLE;
      endcase
    end
  end

endmodule
