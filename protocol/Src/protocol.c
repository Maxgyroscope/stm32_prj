/*
 * protocol.c
 *
 *  Created on: Jul 17, 2024
 *      Author: MaksimS
 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "main.h"
#include "protocol.h"

extern uint8_t adc_val_temp_tx[4];
extern volatile uint8_t flag_adc1_ready;

void Transmit_ADC_Temperature_UART(uint8_t* adc_val_temp_tx)
	{
		 uint8_t ii = 0;
		  	  	  	  //проверка всех флагов и передача побайтно 4 байт значений записанных из АЦП.
		  	  	  	  while(ii<4 && (!(LL_USART_GetTransferDirection(USART2) & LL_USART_DIRECTION_TX)))
		  	  	  	  {
		  	  	  		          LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX);
		  	  	  		          LL_USART_ClearFlag_TC(USART2);
		  	  	  		          LL_USART_EnableIT_TC(USART2);
		  	  	  		          LL_USART_EnableIT_TXE(USART2);
		  	  	  		          LL_USART_TransmitData8(USART2, adc_val_temp_tx[ii]);// тут на плате бы проверил, чтобы удостовериться в корректной передачи
		  	  	  		          ii++;
		  	  	  	  }
	                  	    adc_val_temp_tx[0]= 0; adc_val_temp_tx[1]= 0; adc_val_temp_tx[2]= 0; adc_val_temp_tx[3]= 0;//на всякий случай обнулили, чтобы случайно не принять ложный отчёт
	};

//Реакция на КОРРЕКТНЫЙ пакет
void process_pkt(uint8_t cmd, uint8_t *data, size_t len)
{
    static uint32_t cur_offset = 0;
    switch (cmd)
    {
        //Команды "КАКОГО-ТО" протокола
        case 0xFE:
            if (len<1)
            	return;
            //Параметрый команды
            switch (data[0]) {
                case 0x02: //Команда отправки данных температурного АЦП
                	if(flag_adc1_ready)
                	   {
                		flag_adc1_ready = 0;
                		Transmit_ADC_Temperature_UART((uint8_t*) adc_val_temp_tx);
                	   }
                    return;
                case 0x031: //перевод ADF41020 в Power Down и обратно;
                	adf41020_out_altvoltage0_powerdown(0);
                    return;
                case 0x032: //перевод ADF41020 в обратно (Power UP);
                    adf41020_out_altvoltage0_powerdown(1);
                    return;
                case 0x04: //запрос текущего состояния Lock Detect;
                    //необходимо тестирование
                return;
                case 0x08: // установку / запрос частоты (в МГц), формируемой ADF41020;
                	//adf41020_set_freq(struct adf41020_state *st, uint64_t freq);//необходимо тестирование
                return;
            }
    }
};

//Протокол команд по USART, предположим что он такой))
void proto_process_byte(uint8_t c)
{
	static uint8_t state = 0;
		                 static uint32_t last_tick = 0;
		                 static uint8_t len;
		                 static uint16_t addr;
		                 static uint8_t cmd;
		                 static uint8_t data_len;
		                 static uint8_t crc;
		                 static uint8_t data[240];

		                 //Автомат состояний для парсинга пакета данных с порта
		                 switch (state) {
		                     case 0: //ДЛИНА
		                         if (c<3) return;
		                         crc = len = c;
		                         addr=0;
		                         ++state;
		                         break;
		                     case 1: //Первый байт адреса
		                         addr = c;
		                         crc ^= c;
		                         ++state;
		                         break;
		                     case 2: //Второй байт адреса
		                         addr <<= 8;
		                         addr |= c;
		                         //Если ВСЕМ, то как бы и нам
		                         if (addr == 65534) {
		                           state = 3;
		                           cmd = 0x49;
		                           data[0] = 0x43;
		                           data_len = 3;
		                         } else //А нам ли?

		                         crc ^= c;
		                         ++state;
		                         break;
		                     case 3: // Команда
		                         cmd = c;
		                         crc ^= c;
		                         data_len = 0;

		                         ++state;
		                         break;
		                     default: //Данные команды - они длиной ДЛИНА минус накладные
		                         if (data_len<len-4) {
		                             // printf("DATA: %02x\n", c);
		                             data[data_len++] = c;
		                             crc ^= c;
		                         } else {
		                             //CRC
		                             // printf("CRC: %02x %02x\n", c, crc);

		                             //Данные кончились - последний байт это CRC
		                             if (crc == c) {
		                                 process_pkt(cmd, data, data_len);
		                             }
		                             state = 0;
		                         }
		                         break;
		                 }
}
