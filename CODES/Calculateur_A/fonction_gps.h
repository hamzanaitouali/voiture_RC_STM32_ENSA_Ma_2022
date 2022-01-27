/*
 * fonction_gps.h
 */

#ifndef INC_FONCTION_GPS_H_
#define INC_FONCTION_GPS_H_
#include "stm32f4xx_hal.h"

void GPS_encode(uint8_t *buff_in , short size_buff ,char* cordonnes , char *time );
void GPS_affichage_UART(UART_HandleTypeDef *huart,char* cord ,char* temps);
void trame_to_cordonnes_dec(char* , char* );

#endif /* INC_FONCTION_GPS_H_ */
