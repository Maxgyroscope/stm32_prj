/*
 * protocol.h
 *
 *  Created on: Jul 17, 2024
 *      Author: MaksimS
 */

#ifndef PROTOCOL_INC_PROTOCOL_H_
#define PROTOCOL_INC_PROTOCOL_H_


#include <stdint.h>

void proto_set_myaddr(uint16_t addr);
void proto_process_byte(uint8_t c);
void Transmit_ADC_Temperature_UART(uint8_t* adc_val_temp_tx);

#endif /* PROTOCOL_INC_PROTOCOL_H_ */
