//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Модуль формирование тактовой для блока MDC
// Раздел "10.6.1 MDC/MDIO Timing" документа RTL8211E/RTL8211EGВ datasheet
//=================================================================================
module mdc_clk(
   input       clk_i,      // 50 MHz clock
   input       rst_i,      // Reset signal
   output      mdcclk_o,   // MD clock (T=440ns)
   output      mdsevt_o    // MD event (T~1.85sec)
);

reg        mdcclk;
reg        mdsevt;
reg [4:0]  delay = 5'o0;
reg [21:0] sdelay = 22'b0;
localparam limit = 5'd9;
assign mdcclk_o = mdcclk;
assign mdsevt_o = mdsevt;

always @(posedge clk_i or posedge rst_i) begin
   if(rst_i) begin
      delay <= 5'o0; mdcclk <= 1'b0;
   end
   else begin
      if(delay == limit) begin
         delay <= 5'o0;
         mdcclk <= ~mdcclk;
      end
      else delay <= delay + 1'b1;
   end
end

always @(posedge mdcclk or posedge rst_i) begin
   if(rst_i) begin
      sdelay <= 22'o0; mdsevt <= 1'b0;
   end
   else begin
      if(&sdelay) mdsevt <= 1'b1;
      else			mdsevt <= 1'b0;
      sdelay <= sdelay + 1'b1;
   end
end

endmodule
