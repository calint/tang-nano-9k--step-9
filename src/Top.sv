`timescale 100ps / 100ps
//
`default_nettype none

module Top (
    input wire sys_clk,  // 27 MHz
    input wire sys_rst_n,
    output reg [5:0] led,
    input wire uart_rx,
    output wire uart_tx,
    input wire btn1,

    // magic ports for PSRAM to be inferred
    output wire [ 1:0] O_psram_ck,
    output wire [ 1:0] O_psram_ck_n,
    inout  wire [ 1:0] IO_psram_rwds,
    inout  wire [15:0] IO_psram_dq,
    output wire [ 1:0] O_psram_reset_n,
    output wire [ 1:0] O_psram_cs_n,

    // flash
    output reg  flash_clk,
    input  wire flash_miso,
    output reg  flash_mosi,
    output reg  flash_cs
);

  assign uart_tx = uart_rx;

  // ----------------------------------------------------------
  // -- Gowin_rPLLs
  // ----------------------------------------------------------
  wire rpll_clkout;
  wire rpll_lock;
  wire rpll_clkoutp;
  wire rpll_clkin = sys_clk;

  Gowin_rPLL rpll (
      .clkin(rpll_clkin),  // input clkin 27 MHz
      .lock(rpll_lock),  // output lock
      .clkout(rpll_clkout),  // output 27 MHz
      .clkoutp(rpll_clkoutp)  // output clkout 27 MHz 90 degrees phased
  );

  // ----------------------------------------------------------
  // -- PSRAM_Memory_Interface_HS_V2_Top
  // ----------------------------------------------------------
  wire br_clk_d = sys_clk;
  wire br_memory_clk = rpll_clkout;
  wire br_memory_clk_p = rpll_clkoutp;
  wire br_pll_lock = rpll_lock;
  wire rst_n = sys_rst_n;
  wire [63:0] br_wr_data;
  wire [63:0] br_rd_data;
  wire br_rd_data_valid;
  wire [20:0] br_addr;
  wire br_cmd;
  wire br_cmd_en;
  wire br_init_calib;
  wire br_clk_out;
  wire [7:0] br_data_mask;

  PSRAM_Memory_Interface_HS_V2_Top br (
      .rst_n(rst_n),  // input rst_n
      .clk_d(br_clk_d),  // input clk_d
      .memory_clk(br_memory_clk),  // input memory_clk
      .memory_clk_p(br_memory_clk_p),  // input memory_clk_p
      .clk_out(br_clk_out),  // output clk_out
      .pll_lock(br_pll_lock),  // input pll_lock
      .init_calib(br_init_calib),  // output init_calib

      .cmd(br_cmd),  // input cmd
      .cmd_en(br_cmd_en),  // input cmd_en
      .addr(br_addr),  // input [20:0] addr
      .wr_data(br_wr_data),  // input [63:0] wr_data
      .data_mask(br_data_mask),  // input [7:0] data_mask
      .rd_data(br_rd_data),  // output [63:0] rd_data
      .rd_data_valid(br_rd_data_valid),  // output rd_data_valid

      // inferred PSRAM ports
      .O_psram_ck(O_psram_ck),  // output [1:0] O_psram_ck
      .O_psram_ck_n(O_psram_ck_n),  // output [1:0] O_psram_ck_n
      .IO_psram_dq(IO_psram_dq),  // inout [15:0] IO_psram_dq
      .IO_psram_rwds(IO_psram_rwds),  // inout [1:0] IO_psram_rwds
      .O_psram_cs_n(O_psram_cs_n),  // output [1:0] O_psram_cs_n
      .O_psram_reset_n(O_psram_reset_n)  // output [1:0] O_psram_reset_n
  );

  localparam BURST_RAM_DEPTH_BITWIDTH = 21;

  // ----------------------------------------------------------
  // -- Cache
  // ----------------------------------------------------------
  reg [31:0] cache_address;
  wire [31:0] cache_data_out;
  wire cache_data_out_ready;
  reg [31:0] cache_data_in;
  reg [3:0] cache_write_enable;
  wire cache_busy;

  Cache #(
      .LINE_IX_BITWIDTH(5),  // 1 KB cache (2 ^ 5 * 32 B)
      .RAM_DEPTH_BITWIDTH(BURST_RAM_DEPTH_BITWIDTH),
      .RAM_ADDRESSING_MODE(0)  // addressing 8 bit words
  ) cache (
      .rst(!sys_rst_n || !rpll_lock || !br_init_calib),
      .clk(br_clk_out),

      .address(cache_address),
      .data_in(cache_data_in),
      .write_enable(cache_write_enable),
      .data_out(cache_data_out),
      .data_out_ready(cache_data_out_ready),
      .busy(cache_busy),

      // burst ram wiring; prefix 'br_'
      .br_cmd(br_cmd),
      .br_cmd_en(br_cmd_en),
      .br_addr(br_addr),
      .br_wr_data(br_wr_data),
      .br_data_mask(br_data_mask),
      .br_rd_data(br_rd_data),
      .br_rd_data_valid(br_rd_data_valid)
  );

  assign led[5] = ~cache_busy;

  // ----------------------------------------------------------
  localparam STARTUP_WAIT = 1_000_000;

  // localparam FLASH_TRANSFER_BYTES_NUM = 32'h0020_0000;
  localparam FLASH_TRANSFER_BYTES_NUM = 32'h0010_0000;

  // used while reading flash
  reg [23:0] flash_data_to_send;
  reg [4:0] flash_bits_to_send;
  reg [31:0] flash_counter;
  reg [7:0] flash_current_byte_out;
  reg [7:0] flash_current_byte_num;
  reg [7:0] flash_data_in[4];

  // used while reading flash to increment 'cache_address'
  reg [31:0] cache_address_next;

  localparam STATE_INIT_POWER = 0;
  localparam STATE_LOAD_CMD_TO_SEND = 1;
  localparam STATE_SEND = 2;
  localparam STATE_LOAD_ADDRESS_TO_SEND = 3;
  localparam STATE_READ_DATA = 4;
  localparam STATE_START_WRITE_TO_CACHE = 5;
  localparam STATE_WRITE_TO_CACHE = 6;
  localparam STATE_CACHE_TEST_1 = 7;
  localparam STATE_CACHE_TEST_2 = 8;
  localparam STATE_DONE = 9;
  
  reg [4:0] state = 0;
  reg [4:0] return_state = 0;

  always_ff @(posedge br_clk_out, negedge sys_rst_n) begin
    if (!sys_rst_n || !rpll_lock || !br_init_calib) begin

      flash_counter <= 0;
      flash_clk <= 0;
      flash_mosi <= 0;
      flash_cs <= 1;

      cache_address <= 0;
      cache_address_next <= 0;
      cache_write_enable <= 0;

      led[4:0] <= 5'b1_1111;

      state <= STATE_INIT_POWER;

    end else begin
      case (state)

        STATE_INIT_POWER: begin
          if (flash_counter > STARTUP_WAIT) begin
            flash_counter <= 0;
            state <= STATE_LOAD_CMD_TO_SEND;
          end else begin
            flash_counter <= flash_counter + 1;
          end
        end

        STATE_LOAD_CMD_TO_SEND: begin
          flash_cs <= 0;
          flash_data_to_send[23-:8] <= 3;  // command 3: read
          flash_bits_to_send <= 8;
          state <= STATE_SEND;
          return_state <= STATE_LOAD_ADDRESS_TO_SEND;
        end

        STATE_LOAD_ADDRESS_TO_SEND: begin
          flash_data_to_send <= 0;  // address 0x0
          flash_bits_to_send <= 24;
          flash_current_byte_num <= 0;
          state <= STATE_SEND;
          return_state <= STATE_READ_DATA;
        end

        STATE_SEND: begin
          if (flash_counter == 0) begin
            // at clock to low
            flash_clk <= 0;
            flash_mosi <= flash_data_to_send[23];
            flash_data_to_send <= {flash_data_to_send[22:0], 1'b0};
            flash_bits_to_send <= flash_bits_to_send - 1;
            flash_counter <= 1;
          end else begin
            // at clock to high
            flash_counter <= 0;
            flash_clk <= 1;
            if (flash_bits_to_send == 0) begin
              state <= return_state;
            end
          end
        end

        STATE_READ_DATA: begin
          if (!flash_counter[0]) begin
            flash_clk <= 0;
            flash_counter <= flash_counter + 1;
            if (flash_counter[3:0] == 0 && flash_counter > 0) begin
              // every 16 clock ticks (8 bit * 2)
              flash_data_in[flash_current_byte_num] <= flash_current_byte_out;
              flash_current_byte_num <= flash_current_byte_num + 1;
              if (flash_current_byte_num == 3) begin
                state <= STATE_START_WRITE_TO_CACHE;
              end
            end
          end else begin
            flash_clk <= 1;
            flash_current_byte_out <= {flash_current_byte_out[6:0], flash_miso};
            flash_counter <= flash_counter + 1;
          end
        end

        STATE_START_WRITE_TO_CACHE: begin
          if (!cache_busy) begin
            cache_address <= cache_address_next;
            cache_address_next <= cache_address_next + 4;
            cache_data_in <= {
              flash_data_in[3], flash_data_in[2], flash_data_in[1], flash_data_in[0]
            };
            cache_write_enable <= 4'b1111;
            state <= STATE_WRITE_TO_CACHE;
          end
        end

        STATE_WRITE_TO_CACHE: begin
          if (!cache_busy) begin
            cache_write_enable <= 0;
            flash_current_byte_num <= 0;
            if (cache_address_next < FLASH_TRANSFER_BYTES_NUM) begin
              state <= STATE_READ_DATA;
            end else begin
              flash_cs <= 1;
              state <= STATE_CACHE_TEST_1;
            end
          end
        end

        STATE_CACHE_TEST_1: begin
          if (!cache_busy) begin
            // cache_address <= 0;
            cache_address <= 4;
            cache_write_enable <= 0;
            state <= STATE_CACHE_TEST_2;
          end
        end

        STATE_CACHE_TEST_2: begin
          if (cache_data_out_ready) begin
            // if (cache_data_out == 32'h34_31_32_33) begin  // addr: 0x0
            if (cache_data_out == 32'h63_62_61_0a) begin  // addr: 0x4
              led[4:0] <= 5'b0_0000;
            end else begin
              led[0] <= 1'b0;
            end
            state <= STATE_DONE;
          end
        end

        STATE_DONE: begin
        end

      endcase
    end
  end

endmodule

`default_nettype wire
