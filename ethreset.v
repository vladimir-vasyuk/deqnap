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
reg         reset = 1'b0;
wire        start;
assign e_reset_o = reset;

// Формирование сигнала старта по переднему фронту
reg prev_sig;
always @(posedge clk_i)
   prev_sig <= rst_i;
assign start = ~prev_sig & rst_i;

// Формирование сигнала сброса
always @(posedge clk_i) begin
   if(start) reset <= 1'b1;
   else if(to[19]) reset <= 1'b0;
end

// Счет с обнудением
always @(posedge clk_i, posedge start) begin
   if(start) to <= 20'b0;
   else		 to <= to + 1'b1;
end

endmodule


// =============================================================================
// Модуль формирования сигнала сброса тактового домена Ethernet по комбинированному
// сигналу сброса тактового домена шины.
//
// SDC constraint
//set_false_path -from [get_registers {*rst_sync_eth*eth_rst[1]*}] \
//               -to   [get_registers {*rst_sync_eth*rst_sig*}]
//set_false_path -from [get_registers {*rst_sync_eth*rst_sig*}] \
//               -to   [get_registers {*rst_sync_eth*eth_rst*}]
// =============================================================================
 
`default_nettype none
 
module rst_sync_eth(
    input  wire       wb_clk_i,   // тактовая частота шины
    input  wire       wb_rst_i,   // сигнал сброса
    input  wire       eth_clk_i,  // тактовая частота Ethernet
    output wire       eth_rst_o   // сигнал сброса домена Ethernet
);

reg       rst_sig = 1'b1;
reg [1:0] eth_rst = 2'b0;
 
always @(posedge wb_clk_i, posedge wb_rst_i) begin
   if(wb_rst_i)
      rst_sig <= 1'b0;
   else if(eth_rst[1])
      rst_sig <= 1'b1;
end
 
always @(posedge eth_clk_i) begin
   eth_rst[0] <= ~rst_sig;
   eth_rst[1] <= eth_rst[0];
end
assign eth_rst_o = |eth_rst;

endmodule

`default_nettype wire
