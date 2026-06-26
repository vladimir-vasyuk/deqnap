//=================================================================================
// Реализация контроллера Ethernet DEQNA/DELQA.
//---------------------------------------------------------------------------------
// Модуль программного сброса.
//=================================================================================
module soft_reset(
	input			clk_i,	   // Тактовая
	input			rst_i,	   // Сигнал сброса
	input			csr_sr_i,	// Требование программного сброса
	output		block_o,    // Сигнал блокировки внешней шины
	output		reset_o		// Сигнал программного сброса
);

reg  [1:0]	reset_r = 2'b0;
wire combrst = rst_i | reset_r[1];
assign reset_o = reset_r[1];
assign block_o = reset_r[0];

always @(posedge clk_i) begin
	if(combrst)
		reset_r <= 2'b0;
	else begin
		if(csr_sr_i)
			reset_r[0] <= 1'b1;
		else begin
			if(reset_r[0])
				reset_r[1] <= 1'b1;
		end
	end
end

endmodule
