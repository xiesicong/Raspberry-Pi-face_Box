/**
  ******************************************************************************
  * @file    bsp_SysTick.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   SysTick ϵͳ�δ�ʱ��10us�жϺ�����,�ж�ʱ����������ã�
  *          ���õ��� 1us 10us 1ms �жϡ�     
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
	
#include "bsp_SysTick.h"


static __IO u32 TimingDelay = 0;
 
 
/**
  * @brief  ����ϵͳ�δ�ʱ�� SysTick
  * @param  ��
  * @retval ��
  */
void SysTick_Init( void )
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
	if ( SysTick_Config ( SystemCoreClock / 1000000 ) )	// ST3.5.0��汾
	{ 
		/* Capture error */ 
		while (1);
	}
	
	// �رյδ�ʱ��  
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
	
}

/**
  * @brief   us��ʱ����,1usΪһ����λ
  * @param  
  *		@arg nTime: Delay_us( 1 ) ��ʵ�ֵ���ʱΪ 1 * 1us = 1us
  * @retval  ��
  */
void Delay_us ( __IO u32 nTime )
{ 
	TimingDelay = nTime;	

	// ʹ�ܵδ�ʱ��  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while( TimingDelay != 0 );
	
	// �رյδ�ʱ��  
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;	
	
}

/**
  * @brief   ms��ʱ����,1msΪһ����λ
  * @param  
  *		@arg nTime: Delay_ms( 1 ) ��ʵ�ֵ���ʱΪ 1 * 1ms = 1ms
  * @retval  ��
  */
void Delay_ms(unsigned int ms)
{ 
	unsigned char i;
	for(i=0;i<ms;ms--)
		{
		Delay_us(1000);
		}
}

/**
  * @brief   1s��ʱ����
  * @param  
  *		@arg nTime: Delay_s( 1 ) ��ʵ�ֵ���ʱΪ 1 000* 1ms = 1s
  * @retval  ��
  */
void Delay_s(unsigned int ms)
{ 
	unsigned char i;
	for(i=0;i<ms;ms--)
		{
		Delay_ms(1000);
		}
}



/**
  * @brief  ��ȡ���ĳ���
  * @param  ��
  * @retval ��
  * @attention  �� SysTick �жϺ��� SysTick_Handler()����
  */
void TimingDelay_Decrement(void)
{
	if ( TimingDelay != 0x00 )
	{ 
		TimingDelay --;
	}
	
}


/*********************************************END OF FILE**********************/
