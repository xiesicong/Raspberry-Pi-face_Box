/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   RFID-RC522模块实验
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 F103-MINI STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
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
  * @brief  测试函数
  * @param  无
  * @retval 无
  */
void IC_test ( void )
{
	char cStr [ 30 ];
  uint8_t ucArray_ID [ 4 ];   /*先后存放IC卡的类型和UID(IC卡序列号)*/                                                                                          
	uint8_t ucStatusReturn;     /*返回状态 */                                                                                        
  static uint8_t ucLineCount = 0; 
	
  while ( 1 )
  { 
    /*寻卡*/
		if ( ( ucStatusReturn = PcdRequest ( PICC_REQALL, ucArray_ID ) ) != MI_OK )  
       /*若失败再次寻卡*/
			ucStatusReturn = PcdRequest ( PICC_REQALL, ucArray_ID );		                                                

		if ( ucStatusReturn == MI_OK  )
		{
      /*防冲撞（当有多张卡进入读写器操作范围时，防冲突机制会从其中选择一张进行操作）*/
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
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main ( void )
{
  /*滴答时钟初始化*/
	SysTick_Init ();   
	
  /*USART1 配置模式为 115200 8-N-1，中断接收 */
	USART1_Config ();  
	USART2_Config();
	USART3_Config();
	USART4_Config();
	
	i2c_CfgGpio();				 /*I2C总线的GPIO初始化*/
	OLED_Init();					 /* OLED初始化 */
	
  /*RC522模块所需外设的初始化配置*/
	RC522_Init ();     
	

	printf ( "WF-RC522 Test\r\n" );
	USART2_SendString( DEBUG_USART2,"蓝牙连接正常\n");

	
	PcdReset ();
  
  /*设置工作方式*/
	M500PcdConfigISOType ( 'A' );
	
	OLED_CLS();
	OLED_ShowStr(0,0,(unsigned char*)"RespbettyPi 4B",2);				//测试8*16字符
	OLED_ShowStr(0,2,(unsigned char*)"192.168.137.22",2);				//测试8*16字符
	OLED_ShowStr(0,4,(unsigned char*)"STM32F103RCT6",2);				//测试8*16字符

	
	Delay_s(1);
	
  while ( 1 )
  {
    /*IC卡检测	*/
    IC_test ();
  }	
}



/****************************END OF FILE**********************/

