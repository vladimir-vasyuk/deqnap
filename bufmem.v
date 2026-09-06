//=================================================================================
// DEQNA/DELQA Ethernet controller implementation based on M4 processor (LSI-11M)
// Various memory modules
//
//=================================================================================
// Register file module
//=================================================================================
module regf #(parameter NUM=6)
(
	input						clk_i,   // Clock signal
	input  [NUM/2-1:0]	addr_i,  // Address
	input  [15:0]			data_i,  // Input data
	input						we_i,    // Write enable
	output [15:0]			q_o      // Output data
);

reg [15:0]	x[NUM-1:0];
wire        ena;
assign ena = (addr_i < NUM)? 1'b1 : 1'b0;
assign q_o = ena? x[addr_i] : 16'b0;

always @(posedge clk_i) begin
   if(we_i & ena) begin
      x[addr_i] <= data_i;
   end
end
endmodule


//=================================================================================
// BDL module
//=================================================================================
module bdl(
// Internal bus
   input          wb_clk_i,   // Bus clock signal
   input          wb_rst_i,   // Reset
   input  [2:0]   wb_adr_i,   // Address
   input  [15:0]  wb_dat_i,   // Input data
   input          wb_cyc_i,   // Bus cycle start
   input          wb_we_i,    // Write enable (0 = read)
//   input  [1:0]   wb_sel_i,   // Byte select for write
   input          wb_stb_i,   // Bus cycle strobe
   output         wb_ack_o,   // Device select acknowledge
// DMA bus
   input          dma_inca_i, // DMA address increment
   input  [15:0]  dma_dat_i,  // Input data
   input          dma_we_i,   // Write enable (0 = read)
   input          dma_stb_i,  // Strobe
// Common data output
	output [15:0]	bdl_dat_o
);

// Bus exchange control signals
wire			bus_strobe, bus_write_req; //bus_read_req;
assign bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o;	// Bus cycle strobe
//assign bus_read_req = bus_strobe & ~wb_we_i;			// Read request
assign bus_write_req = bus_strobe & wb_we_i;				// Write request


// Acknowledge signal generation
reg			ack;
always @(posedge wb_clk_i)
   if (wb_stb_i & wb_cyc_i)
		ack <= 1'b1;
   else
		ack <= 1'b0;
assign wb_ack_o = ack & wb_stb_i;

wire [15:0]	bdldin;			// BDL input data
wire [15:0]	bdldout;			// BDL output data
wire [2:0]	bdladr;			// BDL address
wire			bdlwe;			// BDL write signal
reg  [2:0]  dma_adr;

assign bdlwe = dma_stb_i? dma_we_i : (wb_stb_i? wb_we_i : 1'b0);
assign bdladr = dma_stb_i? dma_adr : wb_adr_i;
assign bdldin = dma_stb_i? dma_dat_i : wb_dat_i;
assign bdl_dat_o = bdldout;

always @(posedge wb_clk_i, posedge wb_rst_i)  begin
   if(wb_rst_i) begin
   // Reset
      dma_adr <= 3'b0;
   end
   else  begin
		if(bus_write_req) begin
			case (wb_adr_i[2:0])
            3'b111: // 24016
               dma_adr <= wb_dat_i[2:0];
            default: ; // other addresses not used
         endcase
      end
      else if(dma_inca_i & dma_stb_i) begin
         dma_adr <= dma_adr + 1'b1;
      end
   end
end

regf #(.NUM(6)) bdl(
   .clk_i(wb_clk_i),
   .addr_i(bdladr),
   .data_i(bdldin),
   .we_i(bdlwe),
   .q_o(bdldout)
);
endmodule


//=================================================================================
// Модуль ROM
//=================================================================================
module firmrom(
	input          wb_clk_i,	// тактовая частота шины
	input  [15:0]  wb_adr_i,	// адрес
	output [15:0]  wb_dat_o,	// выходные данные
	input          wb_cyc_i,	// начало цикла шины
	input          wb_stb_i,	// строб цикла шины
	output         wb_ack_o		// подтверждение выбора устройства
);

// Формирование сигнала подтверждения выбора устройства
reg  [1:0] ack;
always @(posedge wb_clk_i)
begin
   ack[0] <= wb_cyc_i & wb_stb_i;
   ack[1] <= wb_cyc_i & ack[0];
end
assign wb_ack_o = wb_cyc_i & wb_stb_i & ack[1];

// Блок ПЗУ
rom bdrom(
   .address(wb_adr_i[11:1]),
   .clock(wb_clk_i),
   .q(wb_dat_o)
);
endmodule


//=================================================================================
// RXBUF module (Receive channel FIFO)
//
// Read: 
//    BA (base address) - memory value + incr. address
//    BA+2              - address value
//    BA+4              - error & flags + byte counter
// Write:
//    BA+2              - address value
//=================================================================================
module rxbuf(
// Domain WB
   input          wb_clk_i,   // clock
   input          wb_rst_i,   // reset
   input  [1:0]   wb_adr_i,	// module address
   input  [15:0]  wb_dat_i,   // input data
   output [15:0]  wb_dat_o,   // output data
   input          wb_cyc_i,   // cycle signal
   input          wb_we_i,    // [1]=write, [0]=read
   input  [1:0]   wb_sel_i,   // byte select for write
   input          wb_stb_i,   // strobe  signal
   output         wb_ack_o,   // acknowledge signal
// DMA (Domain WB)
   input          dma_stb_i,  // strobe
   input          dma_inca_i, // address register increment
   output [15:0]  dma_dat_o,  // output data
// Domain Ethernet
   input          eth_clk_i,  // clock
   input          eth_rst_i,  // reset
   input	 [7:0]   eth_dat_i,  // input data
   input	 [15:0]  eth_cnt_i,  // input byte counter & flags
   input          eth_dwe_i,  // FIFO write enable
   input          eth_cwe_i,  // FIFOcntf write enable
   input          eth_crc_i,  // skip CSR bytes
   input          eth_fls_i,  // flash current frame
// State signals
   output         dat_rdy_o,  // FIFO is not empty
   output         cnt_rdy_o,  // FIFOcntf is not empty
   output         fifo_wen_o  // FIFO write enable
);

// Write pointer: binary, gray, write address, wfull
reg  [12:0] wptr_bin,  wptr_bin_pre, wptr_gray;
reg  wfull;
wire [12:0] wptr_bin_adv, wptr_bin_next, wptr_gray_next;
wire [11:0] waddr = wptr_bin[11:0];
assign wptr_bin_adv   = wptr_bin + eth_we;
assign wptr_bin_next  = eth_crc_i ? (wptr_bin - 13'd4) :
                        eth_fls_i ? wptr_bin_pre : wptr_bin_adv;
assign wptr_gray_next = wptr_bin_next ^ (wptr_bin_next >> 1);

// Read pointer: binary, gray, read address, rempty
reg  [12:0] rptr_bin,  rptr_gray, rptr_bin_next;
reg  rempty;
wire [12:0] rptr_bin_adv, rptr_gray_next;
wire [10:0] raddr = rptr_bin[11:1];
assign rptr_bin_adv  = rptr_bin + ((r_en & ~rempty) ? 13'd2 : 13'd0);

// Restore/assign read next pointer
always @(*) begin
   if(bus_write_req & (wb_adr_i[1:0] == 2'b01)) begin
      rptr_bin_next[12:8] = wb_sel_i[1] ? wb_dat_i[12:8] : rptr_bin[12:8];
      rptr_bin_next[7:0]  = wb_sel_i[0] ? wb_dat_i[7:0]  : rptr_bin[7:0];
   end
   else
      rptr_bin_next = rptr_bin_adv;
end
assign rptr_gray_next = rptr_bin_next ^ (rptr_bin_next >> 1);

// *** Gray‑pointers synchro ***
// read pointer for Ether‑domain
reg [12:0] g_rptr_sync1, g_rptr_sync2;
always @(posedge eth_clk_i, posedge eth_rst_i) begin
   if(eth_rst_i) begin
      g_rptr_sync1 <= 13'b0;
      g_rptr_sync2 <= 13'b0;
   end else begin
      g_rptr_sync1 <= rptr_gray;
      g_rptr_sync2 <= g_rptr_sync1;
   end
end

// write pointer for WB‑domain
reg [12:0] g_wptr_sync1, g_wptr_sync2;
always @(posedge wb_clk_i, posedge wb_rst_i) begin
   if(wb_rst_i) begin
      g_wptr_sync1 <= 13'b0;
      g_wptr_sync2 <= 13'b0;
   end else begin
      g_wptr_sync1 <= wptr_gray;
      g_wptr_sync2 <= g_wptr_sync1;
   end
end
// *********************************

// *** rempty & wfull generation ***
wire rempty_next;
assign rempty_next = (g_wptr_sync2 == rptr_gray_next);

always @(posedge wb_clk_i, posedge wb_rst_i) begin
   if(wb_rst_i)
      rempty <= 1'b1;
   else
      rempty <= rempty_next;
end

wire wfull_next;
assign wfull_next = (wptr_gray_next == {~g_rptr_sync2[12:11],
                     g_rptr_sync2[10:0]});
always @(posedge eth_clk_i, posedge eth_rst_i) begin
   if(eth_rst_i)
      wfull <= 1'b0;
   else
      wfull <= wfull_next;
end
// *********************************

// Bus qualifier signals
wire bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o;	
wire bus_read_req = bus_strobe & ~wb_we_i;
wire bus_write_req = bus_strobe & wb_we_i;

// Acknowledge signal generator (2 wait states)
reg  [1:0] ack;
always @(posedge wb_clk_i) begin
   ack[0] <= wb_cyc_i & wb_stb_i;
   ack[1] <= wb_cyc_i & ack[0];
end
assign wb_ack_o = wb_cyc_i & wb_stb_i & ack[1];

reg  [15:0] data;          // Output data reg.
wire [15:0] buf_data;      // FIFO output data
wire [15:0] cnt_data;      // FIFOcntf output data
assign wb_dat_o = data;
assign dma_dat_o = buf_data;

wire cnt_wf, cnt_re;       // FIFOcntf full & empty signals
assign dat_rdy_o = ~rempty;
assign cnt_rdy_o = ~cnt_re;

`ifdef rx_single_frame
// Double-flop rempty and cnt_re into eth_clk_i domain.
reg rempty_s1, rempty_s2, cnt_re_s1, cnt_re_s2;
always @(posedge eth_clk_i, posedge eth_rst_i) begin
   if (eth_rst_i) begin
      rempty_s1 <= 1'b1; rempty_s2 <= 1'b1;
      cnt_re_s1 <= 1'b1; cnt_re_s2 <= 1'b1;
   end else begin
      rempty_s1 <= rempty;   rempty_s2 <= rempty_s1;
      cnt_re_s1 <= cnt_re;   cnt_re_s2 <= cnt_re_s1;
   end
end
wire drained_sync = rempty_s2 & cnt_re_s2;   // both FIFOs confirmed fully consumed

// *** NEW: rx_hold — stop accepting frames until driver has drained both FIFOs ***
reg rx_hold;
always @(posedge eth_clk_i, posedge eth_rst_i) begin
   if (eth_rst_i)
      rx_hold <= 1'b0;
   else if (eth_cnt_op)        // frame's descriptor entry just landed successfully
      rx_hold <= 1'b1;
   else if (drained_sync)      // driver has fully read out the previous frame
      rx_hold <= 1'b0;
end
assign fifo_wen_o = ~wfull & ~cnt_wf & ~rx_hold;
`else
assign fifo_wen_o = ~wfull & ~cnt_wf;
`endif


wire dma_inca, eth_we, eth_cnt_op;
assign dma_inca = dma_inca_i & dma_stb_i & ~rempty;   // DMA increment address signal
assign eth_cnt_op = eth_cwe_i & ~cnt_wf;              // FIFOcntf write enable 
assign eth_we = eth_dwe_i & ~wfull;                   // FIFO write enable
wire cnt_rrq = bus_read_req & (wb_adr_i[1:0] == 2'b10) & ~cnt_re;
//wire cnt_rrq = bus_read & (wb_adr_i[1:0] == 2'b10) & ~cnt_re;

// *** Read from FIFO ***
// WB read enable generation (1 clock duration)
reg pre_read;
always @(posedge wb_clk_i)
   pre_read <= bus_read_req;
wire bus_read = ~pre_read & bus_read_req;

// FIFO read enable signal
wire r_en = ((wb_adr_i[1:0] == 2'b00) & bus_read) |
            (~bus_read_req & dma_inca);

always @(posedge wb_clk_i, posedge wb_rst_i) begin
   if (wb_rst_i) begin
      rptr_bin  <= 13'b0;
      rptr_gray <= 13'b0;
   end else begin
      rptr_bin  <= rptr_bin_next;
      rptr_gray <= rptr_gray_next;
   end
end

always @(posedge wb_clk_i, posedge wb_rst_i)  begin
   if (wb_rst_i) begin
      data <= 16'b0;
   end
   else begin
      if(bus_read_req) begin
         case (wb_adr_i[1:0])
            2'b00:
               data <= buf_data;
            2'b01:
               data <= {3'b0, rptr_bin};
            2'b10:
               data <= cnt_data;
            default:
               data <= 16'b0;
         endcase
      end
   end
end
// ******************************

// *** FIFO prev. address ***
always @(posedge eth_clk_i, posedge eth_rst_i)  begin
   if (eth_rst_i) begin
      wptr_bin_pre <= 13'b0;
   end
   else begin
      if(eth_cnt_op)
         wptr_bin_pre <= wptr_bin;
   end
end

// *** Write to FIFO ***
always @(posedge eth_clk_i, posedge eth_rst_i) begin
   if (eth_rst_i) begin
      wptr_bin  <= 13'b0;
      wptr_gray <= 13'b0;
   end else begin
      wptr_bin  <= wptr_bin_next;
      wptr_gray <= wptr_gray_next;
   end
end
// ********************************

// RAM
buf2kw bufrx(
   .rdaddress(raddr),
   .wraddress(waddr),
   .rdclock(wb_clk_i),
   .wrclock(eth_clk_i),
   .data(eth_dat_i),
   .wren(eth_we),
   .q(buf_data)
);

// FIFOcntf - To store number of received data bytes & flags
async_fifo #(16, 5) cntrf(
   .wclk(eth_clk_i),
   .wrst(eth_rst_i),
   .w_en(eth_cnt_op),
   .wdata(eth_cnt_i),
   .wfull(cnt_wf),
   .rclk(wb_clk_i),
   .rrst(wb_rst_i),
   .r_en(cnt_rrq),
   .rdata(cnt_data),
   .rempty(cnt_re)
);
endmodule

//=================================================================================
// Async FIFO
//=================================================================================
module async_fifo #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 4            // capacity = 2^4 DATA_WIDTH words
)(
    // write domain
    input  wire                  wclk,    // clock
    input  wire                  wrst,    // reset
    input  wire                  w_en,    // write enable
    input  wire [DATA_WIDTH-1:0] wdata,   // write data
    output reg                   wfull,   // FIFO full

    // read domain
    input  wire                  rclk,    // clock
    input  wire                  rrst,    // reset
    input  wire                  r_en,    // read enable
    output reg  [DATA_WIDTH-1:0] rdata,   // read data
    output reg                   rempty   // FIFO empty
);

localparam PTR_WIDTH = ADDR_WIDTH + 1;          // addttional bit for full/empty recogtition
reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1]; // FIFO memory

reg  [PTR_WIDTH-1:0] wptr_bin,  wptr_gray;      // Write pointer: binary + gray
wire [PTR_WIDTH-1:0] wptr_bin_next, wptr_gray_next;

wire [ADDR_WIDTH-1:0] waddr = wptr_bin[ADDR_WIDTH-1:0];  // Write address

assign wptr_bin_next  = wptr_bin + (w_en & ~wfull);      // Write pointer inc.
assign wptr_gray_next = wptr_bin_next ^ (wptr_bin_next >> 1);

//***  Write operation ***
always @(posedge wclk) begin
   if(w_en && ~wfull)
      mem[waddr] <= wdata;
   end

always @(posedge wclk, posedge wrst) begin
   if (wrst) begin
      wptr_bin  <= {PTR_WIDTH{1'b0}};
      wptr_gray <= {PTR_WIDTH{1'b0}};
   end else begin
      wptr_bin  <= wptr_bin_next;
      wptr_gray <= wptr_gray_next;
   end
end
// ***********************

reg  [PTR_WIDTH-1:0] rptr_bin,  rptr_gray;   // Read pointer: binary + gray
wire [PTR_WIDTH-1:0] rptr_bin_next, rptr_gray_next;

wire [ADDR_WIDTH-1:0] raddr = rptr_bin[ADDR_WIDTH-1:0];  // Read address

assign rptr_bin_next  = rptr_bin + (r_en & ~rempty);     // Read pointer inc.
assign rptr_gray_next = rptr_bin_next ^ (rptr_bin_next >> 1);

//***  Read operation ***
always @(posedge rclk, posedge rrst) begin
   if (rrst)
      rdata <= {DATA_WIDTH{1'b0}};
   else
      rdata <= mem[raddr];
end

always @(posedge rclk, posedge rrst) begin
   if (rrst) begin
      rptr_bin  <= {PTR_WIDTH{1'b0}};
      rptr_gray <= {PTR_WIDTH{1'b0}};
   end else begin
      rptr_bin  <= rptr_bin_next;
      rptr_gray <= rptr_gray_next;
   end
end
// **********************

// *** Gray pointers sync. ***
// rptr_gray -> wclk domain
// wptr_gray -> rclk domain

reg [PTR_WIDTH-1:0] g_rptr_sync1, g_rptr_sync2;
always @(posedge wclk, posedge wrst) begin
   if (wrst) begin
      g_rptr_sync1 <= {PTR_WIDTH{1'b0}};
      g_rptr_sync2 <= {PTR_WIDTH{1'b0}};
   end else begin
      g_rptr_sync1 <= rptr_gray;
      g_rptr_sync2 <= g_rptr_sync1;
   end
end

reg [PTR_WIDTH-1:0] g_wptr_sync1, g_wptr_sync2;
always @(posedge rclk, posedge rrst) begin
   if (rrst) begin
      g_wptr_sync1 <= {PTR_WIDTH{1'b0}};
      g_wptr_sync2 <= {PTR_WIDTH{1'b0}};
   end else begin
      g_wptr_sync1 <= wptr_gray;
      g_wptr_sync2 <= g_wptr_sync1;
   end
end
// ***************************

// EMPTY signal generation
// rempty = (gray write pointer == next gray read pointer)
wire rempty_next;
assign rempty_next = (g_wptr_sync2 == rptr_gray_next);
always @(posedge rclk, posedge rrst) begin
   if (rrst)
      rempty <= 1'b1;
   else
      rempty <= rempty_next;
end

// FULL signal generation
wire wfull_next;
assign wfull_next = (wptr_gray_next == {~g_rptr_sync2[PTR_WIDTH-1:PTR_WIDTH-2],
                     g_rptr_sync2[PTR_WIDTH-3:0]});
always @(posedge wclk, posedge wrst) begin
   if (wrst)
      wfull <= 1'b0;
   else
      wfull <= wfull_next;
end
endmodule


//=================================================================================
// Модуль TXBUF
//=================================================================================
module txbuf(
// Domain WB
	input				wb_clk_i,	// clock
   input          wb_rst_i,   // reset
//	input  [1:0]	wb_adr_i,	// module address
	input  [15:0]	wb_dat_i,	// input data
//	output [15:0]	wb_dat_o,	// output data
	input				wb_cyc_i,	// cycle signal
	input				wb_we_i,		// [1]=write, [0]=read
	input				wb_stb_i,	// strobe  signal
	output			wb_ack_o,	// ack. signal
// DMA (domain WB)
	input				dma_stb_i,	// DMA strobe
	input  [15:0]	dma_dat_i,	// input data
	input				dma_we_i,	// [0]=read, [1]=write
// Domain Ethernet
   input          eth_inca_i, // address register inc.
	output [15:0]	eth_dat_o,	// output data
	input				eth_clk_i,	// clock
   input          eth_rst_i,  // reset
// FIFO signals
   output         ren_o,      // enable read from FIFO
   output         wen_o       // enable write to FIFO
);

// WB qualifier signals
wire bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o;
//wire bus_read_req = bus_strobe & ~wb_we_i; 
wire bus_write_req = bus_strobe & wb_we_i;

// WB ack generator (2 wait states)
reg  [1:0] ack;
always @(posedge wb_clk_i) begin
   ack[0] <= wb_cyc_i & wb_stb_i;
   ack[1] <= wb_cyc_i & ack[0];
end
assign wb_ack_o = wb_cyc_i & wb_stb_i & ack[1];

// WB write enable generation (1 clock duration)
reg pre_write;
always @(posedge wb_clk_i)
 pre_write <= bus_write_req;
wire bus_we = ~pre_write & bus_write_req;

// DMA write enable signal
wire dma_we = dma_stb_i & dma_we_i;

// Selector DMA/WB
wire wren = (dma_stb_i ? dma_we : bus_we) & ~wfull;
wire [15:0] wdata = dma_stb_i ? dma_dat_i : wb_dat_i;

// Write pointer: binary, gray, write address, wfull
reg  [10:0] wptr_bin,  wptr_gray;
reg  wfull;
wire [10:0] wptr_bin_next, wptr_gray_next;
wire [9:0] waddr = wptr_bin[9:0];
//assign wptr_bin_next  = wptr_bin + dma_we;
assign wptr_bin_next  = wptr_bin + wren;
assign wptr_gray_next = wptr_bin_next ^ (wptr_bin_next >> 1);

// Read pointer: binary, gray, read address, rempty
reg  [10:0] rptr_bin,  rptr_gray;
reg  rempty;
wire [10:0] rptr_bin_next, rptr_gray_next;
wire [9:0] raddr = rptr_bin[9:0];
assign rptr_bin_next  = rptr_bin + (eth_inca_i & ~rempty);
assign rptr_gray_next = rptr_bin_next ^ (rptr_bin_next >> 1);

// *** Gray‑pointers synchro ***
// Read pointer for write‑domain
reg [10:0] g_rptr_sync1, g_rptr_sync2;
always @(posedge wb_clk_i, posedge wb_rst_i) begin
   if(wb_rst_i) begin
      g_rptr_sync1 <= 11'b0;
      g_rptr_sync2 <= 11'b0;
   end else begin
      g_rptr_sync1 <= rptr_gray;
      g_rptr_sync2 <= g_rptr_sync1;
   end
end

// Write pointer for read‑domain
reg [10:0] g_wptr_sync1, g_wptr_sync2;
always @(posedge eth_clk_i, posedge eth_rst_i) begin
   if(eth_rst_i) begin
      g_wptr_sync1 <= 11'b0;
      g_wptr_sync2 <= 11'b0;
   end else begin
      g_wptr_sync1 <= wptr_gray;
      g_wptr_sync2 <= g_wptr_sync1;
   end
end
// *****************************

// *** rempty & wfull generation ***
wire rempty_next;
assign rempty_next = (g_wptr_sync2 == rptr_gray_next);

always @(posedge eth_clk_i, posedge eth_rst_i) begin
   if(eth_rst_i)
      rempty <= 1'b1;
   else
      rempty <= rempty_next;
end

wire wfull_next;
assign wfull_next = (wptr_gray_next == {~g_rptr_sync2[10:9],
                     g_rptr_sync2[8:0]});
always @(posedge wb_clk_i, posedge wb_rst_i) begin
   if(wb_rst_i)
      wfull <= 1'b0;
   else
      wfull <= wfull_next;
end

assign wen_o = ~wfull;
assign ren_o = ~rempty;
// *********************************

// *** Read from RAM ***
always @(posedge eth_clk_i, posedge eth_rst_i)  begin
   if (eth_rst_i) begin
      rptr_bin  <= 11'b0;
      rptr_gray <= 11'b0;
   end
   else begin
      rptr_bin  <= rptr_bin_next;
      rptr_gray <= rptr_gray_next;      
   end
end

// *** Write to RAM ***
always @(posedge wb_clk_i, posedge wb_rst_i) begin
   if(wb_rst_i) begin
      wptr_bin  <= 11'b0;
      wptr_gray <= 11'b0;
   end
   else begin
      wptr_bin  <= wptr_bin_next;
      wptr_gray <= wptr_gray_next;
   end
end
// ********************************

// RAM
buf1kw bufwr(
	.rdaddress(raddr),
	.rdclock(eth_clk_i),
	.wraddress(waddr),
	.wrclock(wb_clk_i),
	.data(wdata),
	.wren(wren),
	.q(eth_dat_o)
);
endmodule


//=================================================================================
// Модуль RAM с управляющей программой + BD ROM
//=================================================================================
module firmware (
   input          wb_clk_i,	// тактовая частота шины
   input  [15:0]  wb_adr_i,	// адрес
   input  [15:0]  wb_dat_i,	// входные данные
   output [15:0]  wb_dat_o,	// выходные данные
   input          wb_cyc_i,	// начало цикла шины
   input          wb_we_i,		// разрешение записи (0 - чтение)
   input  [1:0]   wb_sel_i,	// выбор байтов для записи 
   input          prg_stb_i,	// строб модуля RAM
	input          rom_stb_i,	// строб модуля BD ROM
   output         wb_ack_o		// подтверждение выбора устройства
);

wire [1:0]	enaprg;
assign enaprg = prg_stb_i? (wb_we_i ? wb_sel_i : 2'b11) : 2'b00;

wire [15:0]	prgdat, romdat;
assign wb_dat_o = rom_stb_i? romdat : prgdat;

// Формирование сигнала подтверждения выбора устройства
reg  [1:0] prgack;
always @(posedge wb_clk_i) begin
   prgack[0] <= wb_cyc_i & prg_stb_i;
   prgack[1] <= wb_cyc_i & prgack[0];
end
reg [1:0] romack;
always @(posedge wb_clk_i) begin
	romack[0] <= wb_cyc_i & rom_stb_i;
	romack[1] <= wb_cyc_i & romack[0];
end
assign wb_ack_o = (romack[1] & rom_stb_i) | (prgack[1] & prg_stb_i);

firmw ram(
   .address(wb_adr_i[11:1]),
   .byteena(enaprg),
   .clock(wb_clk_i),
   .data(wb_dat_i),
   .rden(~wb_we_i & wb_cyc_i & prg_stb_i),
   .wren( wb_we_i & wb_cyc_i & prg_stb_i),
   .q(prgdat)
);

rom bdrom(
   .address(wb_adr_i[11:1]),
   .clock(wb_clk_i),
	.rden(~wb_we_i & wb_cyc_i & rom_stb_i),
   .q(romdat)
);
endmodule
