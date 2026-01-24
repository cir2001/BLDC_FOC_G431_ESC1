#ifndef __USART_H
#define __USART_H 
#include "sys.h"	
#include <stdio.h>  
//////////////////////////////////////////////////////////////////////////////////	   
//
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_TRANS_LEN     	200  	//定义最大接收字节数 200
#define UART2_DMA_TX_BUF_SIZE 256


void uart1_init(u32 bound);
void uart1_send_byte(u8 data);

void uart2_init(u32 bound);
void uart2_send_byte(u8 data);
void VOFA_Send_UART2(float iq, float id, float ang, float vq, float vd);

#endif	   

