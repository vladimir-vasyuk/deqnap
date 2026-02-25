//=================================================================================
// Реализация контроллера Ethernet DEQNA/DELQA на основе процессора М4 (LSI-11M)
// Всевозможные модули памяти
//
//=================================================================================
// Модуль регистровой памяти
//=================================================================================
module regf #(parameter NUM=6)
(
	input						clk_i,   // тактовая частота
	input  [NUM/2-1:0]	addr_i,  // адрес
	input  [15:0]			data_i,  // выходные данные
	input						we_i,    // разрешение записи
	output [15:0]			q_o      // выходные данные
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
// Модуль RXBUF (FIFO канала приема)
//
// Read: 
//    BA (base address) - memory value + incr. address
//    BA+2              - address value
//    BA+4              - error & flags + byte counter
////    BA+6              - address value (write operation) -- read_only
// Write:
//    BA+2              - address value
//=================================================================================
module rxbuf(
	input          wb_clk_i,   // тактовая частота шины
   input          wb_rst_i,   // сброс
   input  [1:0]   wb_adr_i,	// адрес
   input  [15:0]  wb_dat_i,   // входные данные
	output [15:0]  wb_dat_o,   // выходные данные
	input          wb_cyc_i,   // начало цикла шины
   input          wb_we_i,    // разрешение записи (0 - чтение)
	input          wb_stb_i,   // строб цикла шины
	output         wb_ack_o,   // подтверждение выбора устройства
// ПДП (DMA)
	input          dma_stb_i,  // строб
   input          dma_inca_i, // сигнал инкремента адреса
	output [15:0]  dma_dat_o,  // выходные данные
// Ethernet
   input          eth_clk_i,  // тактовая
	input	 [15:0]  eth_dat_i,  // входные данные пакета
   input	 [15:0]  eth_cnt_i,  // входные данные FIFOcntf
	input          eth_dwe_i,  // разрешение записи данных пакета
   input          eth_cwe_i,  // разрешение записи флагов

   output         fifo_ren_o, // разрешение чтения FIFO
   output         fifo_wen_o  // разрешение запись в FIFO
);

// Write pointer: binary, gray, write address, wfull
reg  [11:0] wptr_bin,  wptr_gray;
reg  wfull;
wire [11:0] wptr_bin_next, wptr_gray_next;
wire [10:0] waddr = wptr_bin[10:0];
assign wptr_bin_next  = wptr_bin + eth_we;
assign wptr_gray_next = wptr_bin_next ^ (wptr_bin_next >> 1);

// Read pointer: binary, gray, read address, rempty
reg  [11:0] rptr_bin,  rptr_gray;
reg  rempty;
wire [11:0] rptr_bin_next, rptr_gray_next;
wire [10:0] raddr = rptr_bin[10:0];
assign rptr_bin_next  = rptr_bin + (r_en & ~rempty);
assign rptr_gray_next = rptr_bin_next ^ (rptr_bin_next >> 1);

// *** Gray‑pointers synchro ***
// read pointer for write‑domain
reg [11:0] g_rptr_sync1, g_rptr_sync2;
always @(posedge eth_clk_i, posedge eth_rst) begin
   if(eth_rst) begin
      g_rptr_sync1 <= 12'b0;
      g_rptr_sync2 <= 12'b0;
   end else begin
      g_rptr_sync1 <= rptr_gray;
      g_rptr_sync2 <= g_rptr_sync1;
   end
end

// write pointer for read‑domain
reg [11:0] g_wptr_sync1, g_wptr_sync2;
always @(posedge wb_clk_i, posedge wb_rst_i) begin
   if(wb_rst_i) begin
      g_wptr_sync1 <= 12'b0;
      g_wptr_sync2 <= 12'b0;
   end else begin
      g_wptr_sync1 <= wptr_gray;
      g_wptr_sync2 <= g_wptr_sync1;
   end
end
// *****************************

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
assign wfull_next = (wptr_gray_next == {~g_rptr_sync2[11:10],
                     g_rptr_sync2[9:0]});
always @(posedge eth_clk_i, posedge eth_rst) begin
   if(eth_rst)
      wfull <= 1'b0;
   else
      wfull <= wfull_next;
end
// *********************************

// Сигналы управления обменом с шиной
wire bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o;	// строб цикла шины
wire bus_read_req = bus_strobe & ~wb_we_i;         // запрос чтения
wire bus_write_req = bus_strobe & wb_we_i;         // запрос записи

// Формирование сигнала подтверждения выбора устройства
reg  [1:0] ack;
always @(posedge wb_clk_i) begin
   ack[0] <= wb_cyc_i & wb_stb_i;
   ack[1] <= wb_cyc_i & ack[0];
end
assign wb_ack_o = wb_cyc_i & wb_stb_i & ack[1];

reg  [15:0] data;          // Регистр выходных данных
wire [15:0] buf_data;      // Выходные данные буферной памяти
wire [15:0] cnt_data;      // Выходные данные FIFOcntf
assign wb_dat_o = data;
assign dma_dat_o = buf_data;

wire cnt_wf, cnt_re;                   // Сигналы full & empty FIFOcntf
assign fifo_wen_o = ~wfull & ~cnt_wf;  // Разрешение записи в буферную память
assign fifo_ren_o = ~rempty & ~cnt_re; // Разрешение чтения буферной памяти

wire dma_inca, eth_we, eth_cnt_op;
assign dma_inca = dma_inca_i & dma_stb_i & ~rempty;   // сигнал инкр. по каналу ППД
assign eth_cnt_op = eth_cwe_i & ~cnt_wf;              // Сигнал записи 
assign eth_we = eth_dwe_i & ~wfull;                   // сигнал записи по каналу ethernet 
wire cnt_rrq = bus_read_req & (wb_adr_i[1:0] == 2'b10) & ~cnt_re;

// Формирование сигнала сброса - домен тактовой ethernet
reg  [1:0]  eth_rstr;
wire        eth_rst;
always @(posedge eth_clk_i) begin
   eth_rstr[0] <= wb_rst_i;
   eth_rstr[1] <= eth_rstr[0];
end
assign eth_rst = eth_rstr[1];

// *** Чтение буферной памяти ***
wire r_en = ((wb_adr_i[1:0] == 2'b00) & bus_read_req) |
            (~bus_read_req & dma_inca);   // Сигнал чтения буферной памяти
always @(posedge wb_clk_i, posedge wb_rst_i)  begin
   if (wb_rst_i) begin
      rptr_bin  <= 12'b0;
      rptr_gray <= 12'b0;
   end
   else begin
      if(bus_write_req & (wb_adr_i[1:0] == 2'b01))
         rptr_bin <= wb_dat_i[11:0];
      else
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
               data <= {4'b0, rptr_bin};
            2'b10:
               data <= cnt_data;
//            2'b11:
//               data <= adr_wc;
//               data <= 16'b0;
         endcase
      end
   end
end
// ******************************

// *** Запись в буферную память ***
always @(posedge eth_clk_i, posedge eth_rst)  begin
   if (eth_rst) begin
      wptr_bin  <= 12'b0;
      wptr_gray <= 12'b0;
   end
   else begin
      if(eth_cnt_op)
         wptr_bin <= wptr_bin - 2'b10; // корректировка 2-х слов CRC
      else
         wptr_bin  <= wptr_bin_next;
      wptr_gray <= wptr_gray_next;
   end
end
// ********************************

// Блок памяти
buf2kw bufrx(
   .rdaddress(raddr),
   .wraddress(waddr),
   .rdclock(wb_clk_i),
   .wrclock(eth_clk_i),
   .data(eth_dat_i),
   .wren(eth_we),
   .q(buf_data)
);

// FIFO кол-ва принятых байт и флаги (FIFOcntf)
async_fifo #(16, 5) cntrf(
	.wclk(eth_clk_i),
   .wrst(eth_rst),
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
	input				wb_clk_i,	// тактовая частота шины
   input          wb_rst_i,   // сброс
//	input  [1:0]	wb_adr_i,	// адрес
//	input  [15:0]	wb_dat_i,	// входные данные
//	output [15:0]	wb_dat_o,	// выходные данные
//	input				wb_cyc_i,	// начало цикла шины
//	input				wb_we_i,		// разрешение записи (0 - чтение)
//	input				wb_stb_i,	// строб цикла шины
//	output			wb_ack_o,	// подтверждение выбора устройства
// ПДП (DMA)
	input				dma_stb_i,	// строб
   input          dma_inca_i, // сигнал инкремента адреса
	input  [15:0]	dma_dat_i,	// входные данные
	input				dma_we_i,	// разрешение записи (0 - чтение)
// Ethernet
   input          eth_inca_i, // сигнал инкремента адреса
	output [15:0]	eth_dat_o,	// выходные данные
	input				eth_clk_i,	// тактовая частота
// FIFO
   output         fifo_ren_o, // разрешение работы с FIFO
   output         fifo_wen_o  // разрешение работы с FIFO
);

// Сигналы управления обменом с шиной
//wire bus_strobe = wb_cyc_i & wb_stb_i & ~wb_ack_o;	// строб цикла шины
//wire bus_we = bus_strobe & wb_we_i;						// запрос записи

// Сигналы управления обменом по каналу ПДП
wire			dma_we; //, dma_inca;
assign dma_we = dma_stb_i & dma_we_i & ~wfull;
//assign dma_inca = dma_inca_i & dma_stb_i;

// Формирование сигнала подтверждения выбора устройства
//reg  [1:0] ack;
//always @(posedge wb_clk_i) begin
//   ack[0] <= wb_cyc_i & wb_stb_i;
//   ack[1] <= wb_cyc_i & ack[0];
//end
//assign wb_ack_o = wb_cyc_i & wb_stb_i & ack[1];

// Формирование сигнала сброса - домен тактовой ethernet
reg  [1:0]  eth_rstr;
wire        eth_rst;
always @(posedge eth_clk_i) begin
   eth_rstr[0] <= wb_rst_i;
   eth_rstr[1] <= eth_rstr[0];
end
assign eth_rst = eth_rstr[1];

// Write pointer: binary, gray, write address, wfull
reg  [10:0] wptr_bin,  wptr_gray;
reg  wfull;
wire [10:0] wptr_bin_next, wptr_gray_next;
wire [9:0] waddr = wptr_bin[9:0];
assign wptr_bin_next  = wptr_bin + dma_we;
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
always @(posedge eth_clk_i, posedge eth_rst) begin
   if(eth_rst) begin
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

always @(posedge eth_clk_i, posedge eth_rst) begin
   if(eth_rst)
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

assign fifo_wen_o = ~wfull;            // Разрешение записи в буферную память
assign fifo_ren_o = ~rempty;           // Разрешение чтения буферной памяти
// *********************************

// *** Чтение буферной памяти ***
always @(posedge eth_clk_i, posedge eth_rst)  begin
   if (eth_rst) begin
      rptr_bin  <= 11'b0;
      rptr_gray <= 11'b0;
   end
   else begin
      rptr_bin  <= rptr_bin_next;
      rptr_gray <= rptr_gray_next;      
   end
end

// *** Запись в буферную память ***
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

// Блок памяти
buf1kw bufwr(
	.rdaddress(raddr),
	.rdclock(eth_clk_i),
	.wraddress(waddr),
	.wrclock(wb_clk_i),
	.data(dma_dat_i),
	.wren(dma_we),
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
