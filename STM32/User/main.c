/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   RFID-RC522ģ��ʵ��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-MINI STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "stm32f10x.h"
#include "bsp_SysTick.h"
#include "bsp_usart1.h"
#include "rc522_config.h"
#include "rc522_function.h"
#include <stdbool.h>
#include "bsp_i2c_gpio.h"
#include "OLED_I2C.h"
#include "bsp_usart2.h"
#include "bsp_usart3.h"
#include "bsp_usart4.h"
/**
  * @brief  ���Ժ���
  * @param  ��
  * @retval ��
  */
void IC_test ( void )
{
	char cStr [ 30 ];
  uint8_t ucArray_ID [ 4 ];   /*�Ⱥ���IC�������ͺ�UID(IC�����к�)*/                                                                                          
	uint8_t ucStatusReturn;     /*����״̬ */                                                                                        
  static uint8_t ucLineCount = 0; 
	
  while ( 1 )
  { 
    /*Ѱ��*/
		if ( ( ucStatusReturn = PcdRequest ( PICC_REQALL, ucArray_ID ) ) != MI_OK )  
       /*��ʧ���ٴ�Ѱ��*/
			ucStatusReturn = PcdRequest ( PICC_REQALL, ucArray_ID );		                                                

		if ( ucStatusReturn == MI_OK  )
		{
      /*����ײ�����ж��ſ������д��������Χʱ������ͻ���ƻ������ѡ��һ�Ž��в�����*/
			if ( PcdAnticoll ( ucArray_ID ) == MI_OK )                                                                   
			{
				sprintf ( cStr, "%02X%02X%02X%02X",
                  ucArray_ID [ 0 ],
                  ucArray_ID [ 1 ],
                  ucArray_ID [ 2 ],
                  ucArray_ID [ 3 ]);
				OLED_ShowStr(0,6,(unsigned char*)ucArray_ID+4,2);
				USART2_SendByte( DEBUG_USART2, ucArray_ID [ 0 ]);
				USART2_SendByte( DEBUG_USART2, ucArray_ID [ 1 ]);
				USART2_SendByte( DEBUG_USART2, ucArray_ID [ 2 ]);
				USART2_SendByte( DEBUG_USART2, ucArray_ID [ 3 ]);
				
								
				printf ( "%s\r\n",cStr ); 			
				

				
				ucLineCount ++;
				
				if ( ucLineCount == 10 ) ucLineCount = 0;
							
			}		
		}	
  }		
}


/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main ( void )
{
  /*�δ�ʱ�ӳ�ʼ��*/
	SysTick_Init ();   
	
  /*USART1 ����ģʽΪ 115200 8-N-1���жϽ��� */
	USART1_Config ();  
	USART2_Config();
	USART3_Config();
	USART4_Config();
	
	i2c_CfgGpio();				 /*I2C���ߵ�GPIO��ʼ��*/
	OLED_Init();					 /* OLED��ʼ�� */
	
  /*RC522ģ����������ĳ�ʼ������*/
	RC522_Init ();     
	

	printf ( "WF-RC522 Test\r\n" );
	USART2_SendString( DEBUG_USART2,"������������\n");

	
	PcdReset ();
  
  /*���ù�����ʽ*/
	M500PcdConfigISOType ( 'A' );
	
	OLED_CLS();
	OLED_ShowStr(0,0,(unsigned char*)"RespbettyPi 4B",2);				//����8*16�ַ�
	OLED_ShowStr(0,2,(unsigned char*)"192.168.137.22",2);				//����8*16�ַ�
	OLED_ShowStr(0,4,(unsigned char*)"STM32F103RCT6",2);				//����8*16�ַ�

	
	Delay_s(1);
	
  while ( 1 )
  {
    /*IC�����	*/
    IC_test ();
  }	
}



/****************************END OF FILE**********************/

