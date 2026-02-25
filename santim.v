//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Генерация сигнала BDCOK
// Раздел "3.6.6 Sanity Timer" в документе EK-DELQA-UG-002
//=================================================================================
module santim(
	input			clock_i,		// 2,5 MHz тактовая
	input			gen_i,		// Сигнал генерации
	output		out_o			// Выход, активный 1
);

reg			rbdcok = 1'b0;
reg  [3:0]	bdcok_cnt = 4'b0;
localparam	BDCOK_LIMIT = 10;
assign out_o = rbdcok;

// Генерация отрицательного BDCOK ~4 msec.
always @(posedge clock_i) begin
	if(gen_i) begin
		if(bdcok_cnt != BDCOK_LIMIT) begin
			bdcok_cnt <= bdcok_cnt + 1'b1;
			rbdcok <= 1'b1;
		end
		else
			rbdcok <= 1'b0;
	end
	else begin
		rbdcok <= 1'b0; bdcok_cnt <= 4'b0;
	end
end

endmodule
