#include "mitsubishi_encoder.h"
#include "string.h"


extern UART_HandleTypeDef huart1;
extern uint8_t UART_RX_raw[9];

uint8_t motor_data_response_packet [9]={0x0};
enum encoder_resolution_t encoder_resolution=unknown_resolution;
enum motor_family_t motor_family=unknown_family;
enum motor_formfactor_t motor_formfactor=unknown_formfactor;
uint16_t motor_speed=0;
uint16_t motor_power=0;
char motor_model_string[10]="HC- F 00";

void motor_identification(void){
	//first send 2 packets with 0x92 command, then 8 with 0x7A command (motor data read)
	uint8_t command = 0x92;
	for(uint8_t i=0;i<10;i++){
		if(i>2){command=0x7A;}
		HAL_UART_Transmit(&huart1, &command, 1, 100);
		HAL_Delay(1);
		HAL_UART_Receive_DMA(&huart1, UART_RX_raw, 9);
		HAL_Delay(1);
	}
	//check if encoder sent data back ok
	if(UART_RX_raw[1]!=0x21){encoder_state=encoder_error_no_communication;}
	else{
		encoder_state=encoder_ok;
		//determine motor family and encoder resolution
		if(UART_RX_raw[2]==0x41){encoder_resolution=p131072ppr;motor_family=j2super;motor_model_string[5]='S';}
		else if(UART_RX_raw[2]==0x3D){encoder_resolution=p8192ppr;motor_family=j2;}
		else{encoder_resolution=unknown_resolution;motor_family=unknown_family;}
		//determine form factor
		switch(UART_RX_raw[3]){
		case 0x12:
			motor_formfactor=kf;motor_model_string[3]='K';break;
		case 0x02:
			motor_formfactor=mf;motor_model_string[3]='M';break;
		default:
			motor_formfactor=unknown_formfactor;break;
		}
		//determine speed and power
		motor_power=(UART_RX_raw[4] >> 4)*100;
		motor_model_string[6]=motor_power/100 + '0';//'0' to convert dec to ascii
		motor_speed=(UART_RX_raw[4] & 0x0F)*1000;
		motor_model_string[7]=motor_speed/1000 + '0';//'0' to convert dec to ascii
	}
	//copy whole response to other global array to show on startup screen
	memcpy(&motor_data_response_packet,&UART_RX_raw,9);
}
