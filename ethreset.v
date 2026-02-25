//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Модуль формирование сигнала сброса
// Раздел "6.6 Reset" документа RTL8211E/RTL8211EGВ datasheet
//=================================================================================
module ethreset(
   input	   clk_i,      // 50МГц тактовая
   input	   rst_i,      // Сигнал сброса (кнопка, программный и т.д.)
   output   e_reset_o   // Сигнал сброса для Ethernet (~10.5мсек)
);

reg [19:0]  to;
reg         reset;
wire        start;
assign e_reset_o = reset;

initial begin
   reset <= 1'b0;
end

// Формирование сигнала старта по переднему фронту
reg prev_sig;
always @(posedge clk_i)
   prev_sig <= rst_i;
assign start = ~prev_sig & rst_i;

// Формирование сигнала сброса
always @(posedge clk_i) begin
   if(start) reset <= 1'b1;
   else if(to[19] & to[17]) reset <= 1'b0;
end

// Счет с обнудением
always @(posedge clk_i, posedge start) begin
   if(start) to <= 20'b0;
   else		 to <= to + 1'b1;
end

endmodule
