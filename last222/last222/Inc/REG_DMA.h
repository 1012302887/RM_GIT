#ifndef __DMA_H
#define	__DMA_H	  
#include "main.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//DMA ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 
 

void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,uint8_t chx,uint32_t par,uint32_t mar,uint16_t ndtr);//����DMAx_CHx
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,uint16_t ndtr);	//ʹ��һ��DMA����		   
#endif
