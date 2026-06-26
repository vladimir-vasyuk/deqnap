//=================================================================================
// Реализация контроллера Ethernet на основе приемо-передатчика RTL8211EG
//---------------------------------------------------------------------------------
// Модуль приема кадра данных
//=================================================================================
module ethreceive(
	input					clk_i,      // Синхросигнал
	input					rst_i,      // Сигнал сброса
	input					rxena_i,    // Сигнал разрешения приема
	input      [7:0]	data_i,     // Шина входных данных
	input					rxdv_i,     // Сигнал достоверности данных
   input             nocrc_i,    // Не обрабатывать CRC
	input					rxer_i,     // Сигнал ошибки
	input      [31:0] crc_i,      // Шина CRC
	output     [15:0] rxfrsts_o,  // Статус принятого кадра (ошибки и кол-во байт)
	output reg [15:0]	data_o,     // Шины выходных данных
	output reg			datwe_o,    // Сигнал разрешения записи данных
	output reg			cnfwe_o,    // Сигнал разрешения записи флагов
	output reg			crcen_o,    // Cигнал разрешения вычисления CRC
	output reg			crcre_o,    // Cигнал сброса CRC
	output     [47:0] macdat_o,   // MAC адрес принятого кадра
	output				macrdy_o,   // Сигнал готовности MAC адреса
	input					cmpdon_i,   // Сигнал завершения проверки MAC адреса
	input					cmpres_i    // Результат проверки (0 - false, 1 - true)
);

reg  [2:0]	ic;				// Счетчик
reg  [1:0]	bc;				// Счетчик байтов 
reg  [47:0]	recmac;			// Принятый MAC адрес
reg  [7:0]	bufdat;			// Буфер принятых данных
reg			runt_err;      // Сигнал ошибки размера кадра
reg			crc_err;       // Сигнал ошибки CRC
reg  [5:0]	sigwait;			// таймер ожидания
//reg  [1:0]  spfrtyp;       // Код приятого спец. кадра (ignored by  DEQNA)
reg         cmpmace;       // Результат проверки MAC-адреса
reg  [10:0] rxcntb;        // Счетчик принятых байтов

assign macdat_o = recmac;
assign macrdy_o = ic[2] & ic[1] & ~ic[0];
assign rxfrsts_o = {2'b0, cmpmace, crc_err, runt_err, rxcntb[10:0]};

// Конечный автомат канала приема
localparam IDLE		= 3'd0;
localparam SIX_55		= 3'd1;
localparam SPD_D5		= 3'd2;
localparam RX_DATA	= 3'd3;
localparam RX_LAST	= 3'd4;
localparam RX_CRC		= 3'd5;
localparam CHK_MAC	= 3'd6;
localparam FINISH		= 3'd7;
reg  [3:0]  rx_state;

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
				ic <= 3'o0;												// Сброс счетчика
				bc <= 2'o0;												// Сброс счетчика байтов
				sigwait <= 6'b111111;
				if(rxdv_i & rxena_i) begin                   // Признак принятых данныхи и сигнал разрешения приема
					if(data_i[7:0] == 8'h55) begin				// Данные преамбулы (0x55)?
                  cmpmace <= 1'b1;
						crc_err <= 1'b0;                       // Сброс сигнала ошибки CRC
                  runt_err <= 1'b0;                      // Сброс сигнала ошибки размера кадра
                  rxcntb <= 11'o0000;                    // Начальное значения счетчика приема
						rx_state<=SIX_55;								// Переход к приему преамбулы
					end
					else
						rx_state<=IDLE;
				end
			end
			SIX_55: begin		// Принять еще 6 байтов 0x55
				if(rxer_i == 1'b0) begin
					if ((data_i[7:0] == 8'h55) & (rxdv_i == 1'b1)) begin
						if(ic == 3'd5) begin
							ic <= 3'd0; rx_state <= SPD_D5;
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
			SPD_D5: begin		// Принять байт разделитель (0xd5)
				if(rxer_i == 1'b0) begin
					if((data_i[7:0] == 8'hd5) && (rxdv_i == 1'b1)) begin
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
					runt_err <= 1'b1;                         // Да - установить сигнал ошибки и, ...
               if(ic < 3'd6)                             // ... если MAC не принят, ...
                  rx_state <= FINISH;                    // ... переход на завершение
               else                                      // ... если MAC принят, ...
                  rx_state <= CHK_MAC;                   // ... переход на проверку MAC
				end
				else begin
					if(rxdv_i == 1'b1) begin						// Есть разрешение приема данных?
                  rxcntb <= rxcntb + 1'd1;				// Нет - инкремент счетчика принятых данных.
						if(ic < 3'd6) begin							// Меньше 6 байт?
							recmac <= {data_i[7:0], recmac[47:8]};	// Да - формирование MAC адреса назначения, ...
							ic <= ic + 1'd1;								// ... инкремент счетчика.
						end
						case(bc)
							2'o0: begin
								bufdat <= data_i[7:0];				// Да - данные во внутренний буфер, ...
								bc <= bc + 1'b1;						// ... инкремент счетчика байтов, ...
								datwe_o <= 1'b0;                 // ... запрет записи в буферную память.
							end
							2'o1: begin
								data_o <= {data_i[7:0], bufdat[7:0]};	// ... данные на шину буферной памяти, ...
								datwe_o <= 1'b1;                 // ... разрешение записи в буферную память, ...
								bc <= 2'o0;                      // ... инкремент счетчика байтов, ...
							end   
						endcase
					end
					else begin                                // Нет разрешения приема данных, ...
						datwe_o <= 1'b0;                       // ... отключить запись в буфер, ...
						crcen_o <=1'b0;                        // ... сброс сигнала разрешения вычисления CRC ...
						rx_state <= RX_LAST;                   // ... и на запись оставшихся данных
					end
				end
			end
			RX_LAST: begin
            if(~rxena_i) begin
               runt_err <= 1'b1;
               rx_state <= CHK_MAC;
            end
            else begin
				   case(bc)
					   2'o1: begin                            // Если есть не записанные данные...
						   data_o <= {8'b0,bufdat[7:0]};       // ... записать 
						   datwe_o <= 1'b1;
						   bc <= 2'o0;
					   end
					   2'o0: begin                            // Все данные записаны, ...
						   datwe_o <= 1'b0;                    // ... отключить запись в буфер
                     if(nocrc_i) begin
                        crcen_o  <= 1'b0;
                        rx_state <= CHK_MAC;
                     end
                     else
                        rx_state <= RX_CRC;
                  end
				   endcase
            end
			end
			RX_CRC: begin		// Проверка CRC
				rx_state <= CHK_MAC;                         // На завершение
				rxcntb <= rxcntb - 11'd4;                    // Минус 4 байта (CRC)
				if(crc_i != 32'hC704DD7B) begin              // CRC верен?
					crc_err <= 1'b1;                          // Нет - установить сигнал ошибки
				end
			end
			CHK_MAC: begin                                  // Проверка MAC адреса принятого кадра
				if(cmpdon_i) begin
               cmpmace <= ~cmpres_i;                        // Сохранить результат и ...
               rx_state <= FINISH;                       // ... переход к завершению
				end
				else begin                                   // Ждем сигнал завершения проверки
					sigwait <= sigwait -1'b1;
					if(|sigwait == 1'b0) begin                // Сигнала завершения проверки нет, ...
                  cmpmace <= 1'b1;                        // ... установить признак ошибки ...
						rx_state <= FINISH;                    // ... переход к завершению
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

/*
// Обнаружение спец. кадров
always@(negedge clk_i) begin
   case(rx_state)
      SPD_D5: spfrtyp <= 2'b0;
      RX_DATA: begin
         if(rxcntb == 11'd14) begin
            case(data_o)
               16'o0220: spfrtyp <= 2'b01;   // ECTP frame
               16'o1140: spfrtyp <= 2'b10;   // MOP frame
            endcase
         end
      end
   endcase
end
*/
endmodule
