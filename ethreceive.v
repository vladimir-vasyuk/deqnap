//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Модуль приема кадра данных
//=================================================================================
module ethreceive #(
   parameter MIN_FRAME_LEN = 60     // minimum frame length in bytes (magic number -> parameter)
)(
	input					   clk_i,      // Синхросигнал
	input					   rst_i,      // Сигнал сброса
	input					   rxena_i,    // Сигнал разрешения приема
	input      [7:0]     data_i,     // Шина входных данных
	input					   rxdv_i,     // Сигнал достоверности данных
   input                nocrc_i,    // Не обрабатывать CRC
	input					   rxer_i,     // Сигнал ошибки
	input      [31:0]    crc_i,      // Шина CRC
   input                loop_i,     // Режим loopback
	output     [15:0]    rxfrsts_o,  // Статус принятого кадра (ошибки и кол-во байт)
	output reg [7:0]     data_o,     // Шины выходных данных
	output reg			   datwe_o,    // Сигнал разрешения записи данных
	output reg			   cnfwe_o,    // Сигнал разрешения записи флагов
	output reg           crcen_o,    // Cигнал разрешения вычисления CRC
	output reg			   crcre_o,    // Cигнал сброса CRC
   output reg           skpcrc_o,   // Сигнал пропуска 4 байт 
   output reg           flashd_o,   // Сигнал очистки текущих данных
	output     [47:0]    macdat_o,   // MAC адрес принятого кадра
	output				   macrdy_o,   // Сигнал готовности MAC адреса
	input					   cmpdon_i,   // Сигнал завершения проверки MAC адреса
	input					   cmpres_i    // Результат проверки (0 - false, 1 - true)
);

reg  [2:0]	ic;				      // Счетчик
reg  [47:0]	recmac;			      // Принятый MAC адрес
reg			runt_err;            // Сигнал ошибки размера кадра
reg			crc_err;             // Сигнал ошибки CRC
reg         short_err;
reg  [5:0]	sigwait;			      // таймер ожидания
reg  [10:0] rxcntb;              // Счетчик принятых байтов
reg         odd;                 // Сингал нечетного кол-ва принятых байт

assign macdat_o = recmac;
//assign macrdy_o = ic[2] & ic[1] & ~ic[0];
assign macrdy_o = (ic == 3'd6);
assign rxfrsts_o = {2'b0, short_err, crc_err, runt_err, rxcntb[10:0]};

// Конечный автомат канала приема
localparam IDLE		= 4'd0;
localparam ET_PRM		= 4'd1;
localparam ET_SFD		= 4'd2;
localparam RX_DATA	= 4'd3;
localparam RX_CRC		= 4'd4;
localparam CHK_MIN	= 4'd5;
localparam CHK_ODD	= 4'd6;
localparam CHK_MAC	= 4'd7;
localparam FINISH		= 4'd8;
reg  [3:0]  rx_state;

//Константы
localparam  ETH_CRC = 32'hC704DD7B;
localparam  PREAMBLE = 8'h55;
localparam  SFD = 8'hd5;

// Основной блок
always@(negedge clk_i, posedge rst_i) begin
   if(rst_i) begin
      rx_state <= IDLE;		// Начальное состояние автомата
   end
   else begin
      case(rx_state)
         IDLE: begin			// Состояние ожидания
				crcen_o <= 1'b0;										// Сброс сигнала разрешения вычисления CRC
				crcre_o <= 1'b1;										// Установка сигнала сброса CRCC
				datwe_o <= 1'b0;                             // Сброс сигнала разрешения записи данных
            cnfwe_o <= 1'b0;                             // Сброс сигнала разрешения записи флагов
				ic <= 3'd0;												// Сброс счетчика
				sigwait <= 6'b111111;                        // Счетчик тайм-аут проверки MAC
            skpcrc_o <= 1'b0;                            // Сброс сигнала пропуска CRC
            odd <= 1'b0;
            flashd_o <= 1'b0;
				if(rxdv_i & rxena_i) begin                   // Признак принятых данныхи и сигнал разрешения приема
					if(data_i[7:0] == PREAMBLE) begin         // Данные преамбулы (0x55)?
                  short_err <= 1'b0;
						crc_err <= 1'b0;                       // Сброс сигнала ошибки CRC
                  runt_err <= 1'b0;                      // Сброс сигнала ошибки размера кадра
                  rxcntb <= 11'd0000;                    // Начальное значения счетчика приема
						rx_state <= ET_PRM;							// Переход к приему преамбулы
					end
					else
						rx_state<=IDLE;
				end
			end
			ET_PRM: begin		// Принять еще 6 байтов 0x55
				if(rxer_i == 1'b0) begin
					if ((data_i[7:0] == PREAMBLE) & (rxdv_i == 1'b1)) begin
						if(ic == 3'd5) begin
							ic <= 3'd0; rx_state <= ET_SFD;
						end
						else
							ic <= ic + 1'd1;
					end
					else begin
						rx_state<=IDLE;
					end
				end
				else begin
					rx_state<=IDLE;
				end
			end
			ET_SFD: begin		// Принять байт разделитель (0xd5)
				if(rxer_i == 1'b0) begin
					if((data_i[7:0] == SFD) && (rxdv_i == 1'b1)) begin
						ic <= 3'd0; rx_state <= RX_DATA;
					end
					else begin
						rx_state <= IDLE;
					end
				end
				else begin
					rx_state <= IDLE;
				end
			end
			RX_DATA: begin		// Основные данные
				crcen_o <= 1'b1;                             // Разрешить вычисление CRC
				crcre_o <= 1'b0;                             // Убрать сигнал сброса CRC
				if(rxer_i | ~rxena_i) begin                  // Входная ошибка или заполненное FIFO?
					runt_err <= 1'b1;                         // Да - установить сигнал ошибки, ...
               datwe_o <= 1'b0;                          // ... отключить запись в буфер, ...
					crcen_o <=1'b0;                           // ... сброс сигнала разрешения вычисления CRC,...
               if(ic < 3'd6)                             // Если MAC не принят, =>
                  rx_state <= FINISH;                    // ... переход на завершение.
               else                                      // Если MAC принят =>
                  rx_state <= CHK_MAC;                   // ... переход на проверку MAC
				end
				else begin
					if(rxdv_i == 1'b1) begin						// Есть разрешение приема данных?
                  rxcntb <= rxcntb + 1'd1;				   // Да - инкремент счетчика принятых данных.
						if(ic < 3'd6) begin							// Меньше 6 байт?
							recmac <= {data_i[7:0], recmac[47:8]};	// Да - формирование MAC адреса назначения, ...
							ic <= ic + 1'd1;								// ... инкремент счетчика.
						end
						data_o <= data_i[7:0];                 // Данные на шину буферной памяти, ...
						datwe_o <= 1'b1;                       // ... разрешение записи в буферную память
					end
					else begin                                // Нет разрешения приема данных, ...
						datwe_o <= 1'b0;                       // ... отключить запись в буфер, ...
						crcen_o <=1'b0;                        // ... сброс сигнала разрешения вычисления CRC ...
						rx_state <= RX_CRC;                    // ... и на проверку CRC
					end
				end
			end
			RX_CRC: begin		// Проверка CRC
            if(!nocrc_i) begin                           // Обычный режим
               rx_state <= CHK_MIN;                      // На проверку минимального размера пакета
               rxcntb <= rxcntb - 11'd4;                 // Минус 4 байта (CRC)
               skpcrc_o <= 1'b1;                         // Сигнал пропуска 4 байт в буферной памяти
               if(crc_i != ETH_CRC) begin                // CRC верен?
                  crc_err <= 1'b1;                       // Нет - установить сигнал ошибки
               end
            end
            else                                         // Режим пересылки BD ROM
               rx_state <= FINISH;
         end
			CHK_MIN: begin
            skpcrc_o <= 1'b0;                            // Сброс сигнала пропуска 4 байт
            if (!loop_i && (rxcntb < MIN_FRAME_LEN)) begin
               // Normal mode, frame still short of minimum: pad one zero byte/cycle
//               runt_err <= 1'b1;
               short_err <= 1'b1;
               rxcntb <= rxcntb + 1'b1;
               data_o <= 8'b0;
               datwe_o <= 1'b1;
            end
            else begin
               datwe_o  <= 1'b0;
               rx_state <= CHK_ODD;
            end
         end
         CHK_ODD: begin
            if((rxcntb[0] == 1'b1) && (odd == 1'b0)) begin
               odd <= 1'b1;
               data_o <= 8'b0;
               datwe_o <= 1'b1;
            end
            else begin
               datwe_o  <= 1'b0;
               odd <= 1'b0;
               rx_state <= CHK_MAC;
            end
         end
			CHK_MAC: begin                                  // Проверка MAC адреса принятого кадра
				if(cmpdon_i) begin
               if(cmpres_i)
                  rx_state <= FINISH;                    // ... переход к завершению
               else begin
                  flashd_o <= 1'b1;
                  rx_state <= IDLE;
               end
				end
				else begin                                   // Ждем сигнал завершения проверки
					sigwait <= sigwait -1'b1;
					if(|sigwait == 1'b0) begin                // Сигнала завершения проверки нет, ...
                  flashd_o <= 1'b1;
						rx_state <= IDLE;                      // ... переход к завершению
               end
				end
			end
			FINISH: begin
            cnfwe_o <= 1'b1;
            rx_state <= IDLE;
			end
			default: rx_state <= IDLE;
		endcase
	end
end

endmodule
