//=================================================================================
// Реализация контроллера Ethernet DEQNA/DELQA на основе процессора М4 (LSI-11M)
//=================================================================================
// Модуль генерации сигналов сброса для внутреннего ЦПУ
// Полностью взят из проекта https://github.com/forth32/dvk-fpga
//=================================================================================
`define	DCLO_WIDTH_CLK	5
`define	ACLO_DELAY_CLK	3
//*****************************************************
//* Модуль генерации сигнадов сброса и таймера
//*****************************************************
module cpu_reset (
   input    clk_i,		// тактовая частота 50 МГц
   input    rst_i,		// кнопка сброса, 0 - сброс, 1 - работа
   output   dclo_o,		// авария DC
   output   aclo_o,		// авария AC
   output	irq50_o		// прерывания с частотой 50 Гц
);
localparam DCLO_COUNTER_WIDTH = log2(`DCLO_WIDTH_CLK);
localparam ACLO_COUNTER_WIDTH = log2(`ACLO_DELAY_CLK);

reg [DCLO_COUNTER_WIDTH-1:0] dclo_cnt;
reg [ACLO_COUNTER_WIDTH-1:0] aclo_cnt;
reg [1:0]	reset;
reg [18:0]	intcount;    			// счетчик для генерации прерываний
reg			aclo_out, dclo_out;
reg			irq50;
assign dclo_o = dclo_out;
assign aclo_o = aclo_out;

reg prevs;
always @(posedge clk_i)
	prevs <= irq50;
assign irq50_o = ~prevs & irq50;

always @(posedge clk_i) begin
   //
   // Синхронизация сигнала сброса для предотвращения метастабильности
   //
   reset[0] <= rst_i; 
   reset[1] <= reset[0];
   
   if (reset[1])   begin
      dclo_cnt     <= 0;
      aclo_cnt     <= 0;
      aclo_out      <= 1'b1;
      dclo_out      <= 1'b1;
      intcount    <= 19'd000000;
      irq50       <= 1'b0;
   end
   else  begin
      //
      // Счетчик задержки DCLO
      //
      if (dclo_cnt != `DCLO_WIDTH_CLK)   dclo_cnt <= dclo_cnt + 1'b1;
      else  dclo_out <= 1'b0;  // снимаем DCLO
         
      //
      // Счетчик задержки ACLO
      //
      if (~dclo_out)
         if (aclo_cnt != `ACLO_DELAY_CLK) aclo_cnt <= aclo_cnt + 1'b1;
         else   aclo_out <= 1'b0;
         
      // генерация импульсов прерывания
      if (|intcount == 1'b0) begin
         intcount <= 19'd500000;      
         irq50 <= ~irq50;
      end
      else intcount <= intcount-1'b1;   
  
   end
end

// получение количества двоичных разрядов в  числе
function integer log2(input integer value);
   begin
      for (log2=0; value>0; log2=log2+1) 
         value = value >> 1;
   end
endfunction

endmodule
