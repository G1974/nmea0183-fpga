// uart_tx.sv
// 8N1 UART transmitter with a simple valid/ready handshake.

module uart_tx #(
  parameter int CLK_HZ = 27_000_000,
  parameter int BAUD   = 4800
) (
  input  logic       clk,
  input  logic       rst,
  input  logic [7:0] data,
  input  logic       valid,
  output logic       ready,   // can take a byte this cycle
  output logic       tx
);

  localparam int CLKS_PER_BIT = CLK_HZ / BAUD;
  localparam int CW           = $clog2(CLKS_PER_BIT);

  // {stop, data[7:0], start}, shifted out LSB first
  logic [9:0]    frame;
  logic [3:0]    bits_left;
  logic [CW-1:0] cnt;

  assign ready = (bits_left == 0);
  assign tx    = frame[0];

  always_ff @(posedge clk) begin
    if (rst) begin
      frame     <= '1;           // line idle high
      bits_left <= '0;
      cnt       <= '0;
    end else if (ready) begin
      if (valid) begin
        frame     <= {1'b1, data, 1'b0};
        bits_left <= 4'd10;
        cnt       <= '0;
      end
    end else if (cnt == CW'(CLKS_PER_BIT - 1)) begin
      cnt       <= '0;
      frame     <= {1'b1, frame[9:1]};
      bits_left <= bits_left - 1'b1;
    end else begin
      cnt <= cnt + 1'b1;
    end
  end

endmodule
