#include "fonction_gps.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

// prototypes :
void trame_to_cordonnes_dec(char* , char* );

// end prot
int Calcule_checksum(char *msg) {
		int checksum = 0;
		int j = 0;
		for (j = 1; j < strlen(msg) - 4; j++) {
			checksum = checksum ^ (unsigned) msg[j];
		}
		return checksum;
}

uint32_t p10(uint8_t p){
	uint8_t i; uint32_t puiss=1;
	for(i=1 ; i<=p ;i++)  puiss *= 10;
	return puiss;
}

uint32_t char_to_int(char chaine[] , short ln){
	  uint8_t i=0; uint32_t val=0;
	while(chaine[i] != '\0'){
		val += (chaine[i]-48)* p10(ln-2-i);
		i++;
	}
	return val;
}


void GPS_encode(uint8_t *buff_in , short size_buff ,char* cordonnes , char *time ){ // cord 27 elem / time 7 elem
	char cord_tmp[27]={0};
	char temp[7]={0};

	char Buff_In_Str[size_buff];
	char Trames[80];
    char *Somme_Tr_in;
	char Checksum_trame[3];
	memset(Buff_In_Str, 0, size_buff);
	sprintf(Buff_In_Str, "%s", buff_in);

	// splitting --> strsep()
	char *Ligne, *string;

	string = strdup(Buff_In_Str);
     uint8_t trame_existe=0;
	// splitting par "\n"
	while ((Ligne = strsep(&string, "\n")) != NULL && !trame_existe) {

		memset(Trames, 0, 80);

		sprintf(Trames, "%s", Ligne);

		if ((strstr(Trames, "$GPGLL") != 0) && strlen(Trames) >= 49 && strstr(Trames, "*") != 0) {

			Somme_Tr_in = strstr(Trames, "*");

			memcpy(Checksum_trame, &Somme_Tr_in[1], 2);

			Checksum_trame[2] = '\0';

			uint8_t Checksum_calc = Calcule_checksum(Trames);

			char hex[2];
			// convertion en hexa
			sprintf(hex, "%X", Checksum_calc);
             // verification de checksum
			if (strstr(Checksum_trame, hex) != NULL) {
				trame_existe =1;
				//pour afficher la trame : $GPGLL
				//HAL_UART_Transmit(&huart2, (uint8_t*) Trames, 50, 70);
				//HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, 300);

				memcpy(cord_tmp, &Trames[7], 36);
				cord_tmp[26]='\0';
				memcpy(temp, &Trames[34],6);
				temp[6]='\0';
                strcpy(cordonnes,cord_tmp);
                strcpy(time, temp);
	            //exemple : "$GPGLL,3139.58468,N,00801.40319,W,211821.00,A,A*7B\r\n"
			}
		}
	}
}

// affichage :

void GPS_affichage_UART(UART_HandleTypeDef *huart,char* cord ,char* temps){
	// iput :  31 39.58468,N,00801.40319,W        211821

    char time_str[9]= {0};
    char time_sec[3]= {0};
    char time_min[3]= {0};
    char cord_final[23]= {0};
    trame_to_cordonnes_dec(cord , cord_final);



    memcpy(time_min, &temps[2], 2);
    memcpy(time_sec, &temps[4], 2);
    memcpy(time_str, &temps[0], 2);
    strcat(time_str, ":");
    strcat(time_str, time_min);
    strcat(time_str, ":");
    strcat(time_str, time_sec);

    HAL_UART_Transmit(huart, (uint8_t*) "LAT/LON: ", 9, 80);
    HAL_UART_Transmit(huart, (uint8_t*) cord_final, 23, 80);
    HAL_UART_Transmit(huart, (uint8_t*) "\tTime: ", 7, 80);
    HAL_UART_Transmit(huart, (uint8_t*) time_str, 8, 80);
    HAL_UART_Transmit(huart, (uint8_t*) "\r\n", 2, 80);
}

void trame_to_cordonnes_dec(char* tram_cord, char* cord_out){
	    char N_S = tram_cord[11];
		char E_W = tram_cord[25];
	    int8_t lat_deg , long_deg ,signe=0;
	    char ch[6]={0};
	// lat
	    memcpy(ch, &tram_cord[0], 2); ch[2]='\0';
	    lat_deg = char_to_int( ch , 3);

	    memcpy(ch, &tram_cord[2], 2); ch[2]='\0';
	    int8_t lat_min_int = char_to_int( ch , 3);

	    memcpy(ch, &tram_cord[5], 5); ch[5]='\0';
	    int32_t lat_min_frac = char_to_int( ch , 6);

	    float lat = lat_deg + (lat_min_int + lat_min_frac/100000.0 )/60.0;
	    if(N_S == 'N') signe =1 ; else if(N_S == 'S') signe = -1;
        lat = lat * signe;

       // char lat_str[15];
      //  sprintf(lat_str, "%g", lat);
        gcvt(lat, 9, cord_out);
   // lon
        memcpy(ch, &tram_cord[13], 3); ch[3]='\0';
		long_deg = char_to_int( ch , 4);

		memcpy(ch, &tram_cord[16], 2); ch[2]='\0';
	    lat_min_int = char_to_int( ch , 3);

		memcpy(ch, &tram_cord[19], 5); ch[5]='\0';
		lat_min_frac = char_to_int( ch , 6);

		float longi = long_deg + (lat_min_int + lat_min_frac/100000.0 )/60.0;
		if(E_W == 'E') signe =1 ; else if(E_W == 'W') signe = -1;
		longi = longi * signe;
		 char longi_str[12];
		gcvt(longi, 10, longi_str );
		strcat(cord_out, ",");
		strcat(cord_out, longi_str);
}



