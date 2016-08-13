/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/07
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "interrupt.h"

#include "device.h"

#include "cmsis_os.h"

namespace driver {

struct map_table interrupt::s_map[STM32F4xx_USER_IRQNUM_MAX];

interrupt::interrupt(void)
{

}

interrupt::~interrupt(void)
{

}

/*
 * ��ʼ���ж�
 * @return	�����ɹ����� 0
 */
s32 interrupt::irq_init(void)
{
	/*
	 * ������ռ���ȼ�����Ӧ���ȼ�λ��:
	 * NVIC_PRIORITYGROUP_4: 4λ��ռ���ȼ���0λ��Ӧ���ȼ�
	 */
	/* Set Interrupt Group Priority */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

	return 0;
}

/*
 * �ж���Դ����
 * @param[in]	irq		�豸�ж��߼����
 * @param[in]	handler	�豸�жϷ���������
 * @return	�����ɹ����� 0
 * @note		������������ж��߼�������жϷ������
 */
s32 interrupt::request_irq(s32 irq, irq_handler *handler)
{
	if (irq >= ARRAYSIZE(s_map) || handler == NULL) {
		return -1;
	}

	s_map[irq].irq = irq;
	s_map[irq].handler = handler;

	HAL_NVIC_SetPriority((IRQn_Type)(s_map[irq].irq), 1, 1);

	return 0;
}

void interrupt::free_irq(s32 irq)
{
	if (irq < ARRAYSIZE(s_map)) {
        s_map[irq].irq = -1;
        s_map[irq].handler = NULL;
	}
}

void interrupt::enable_irq(s32 irq)
{
	HAL_NVIC_EnableIRQ((IRQn_Type)(s_map[irq].irq));
}

void interrupt::disable_irq(s32 irq)
{
	HAL_NVIC_DisableIRQ((IRQn_Type)(s_map[irq].irq));
}

void interrupt::enable_all_irq(void)
{

}

void interrupt::disable_all_irq(void)
{

}

/*
 * �ж�����һ���ַ���
 * @note TODO:�ڴ����һ���ж������������ж�������irq_handler����������ʵ��
 */
#ifdef __cplusplus
extern "C" {
#endif


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
#if defined(USE_HAL_DRIVER)
    HAL_IncTick();
#if (USE_STM32F4_DEMO)
    /* Call user callback */
    HAL_SYSTICK_IRQHandler();
#endif
#endif
    osSystickHandler();
}

void USART1_IRQHandler(void)
{
	interrupt::s_map[USART1_IRQn].handler->isr();
}
void DMA2_Stream5_IRQHandler(void)
{
	interrupt::s_map[DMA2_Stream5_IRQn].handler->isr();
}
void DMA2_Stream7_IRQHandler(void)
{
	interrupt::s_map[DMA2_Stream7_IRQn].handler->isr();
}

void USART2_IRQHandler(void)
{
	interrupt::s_map[USART2_IRQn].handler->isr();
}
void DMA1_Stream5_IRQHandler(void)
{
	interrupt::s_map[DMA1_Stream5_IRQn].handler->isr();
}
void DMA1_Stream6_IRQHandler(void)
{
	interrupt::s_map[DMA1_Stream6_IRQn].handler->isr();
}

void USART3_IRQHandler(void)
{
	interrupt::s_map[USART3_IRQn].handler->isr();
}
void DMA1_Stream1_IRQHandler(void)
{
	interrupt::s_map[DMA1_Stream1_IRQn].handler->isr();
}
void DMA1_Stream3_IRQHandler(void)
{
	interrupt::s_map[DMA1_Stream3_IRQn].handler->isr();
}


void DMA2_Stream2_IRQHandler(void)
{
	interrupt::s_map[DMA2_Stream2_IRQn].handler->isr();
}
void DMA2_Stream3_IRQHandler(void)
{
	interrupt::s_map[DMA2_Stream3_IRQn].handler->isr();
}


#if 0
//I2C1_DMA_TX
void DMA1_Channel6_IRQHandler(void)
{
	interrupt::s_map[DMA1_Channel6_IRQn].handler->isr();
}
//I2C1_DMA_RX
void DMA1_Channel7_IRQHandler(void)
{
	interrupt::s_map[DMA1_Channel7_IRQn].handler->isr();
}


//EXTI
void EXTI0_IRQHandler(void)
{
	interrupt::s_map[EXTI0_IRQn].handler->isr();
}
void EXTI1_IRQHandler(void)
{
	interrupt::s_map[EXTI1_IRQn].handler->isr();
}
void EXTI2_IRQHandler(void)
{
	interrupt::s_map[EXTI2_TSC_IRQn].handler->isr();
}
void EXTI3_IRQHandler(void)
{
	interrupt::s_map[EXTI3_IRQn].handler->isr();
}
void EXTI4_IRQHandler(void)
{
	interrupt::s_map[EXTI4_IRQn].handler->isr();
}

#endif

void TIM2_IRQHandler(void)
{
	interrupt::s_map[TIM2_IRQn].handler->isr();
}
void TIM3_IRQHandler(void)
{
	interrupt::s_map[TIM3_IRQn].handler->isr();
}

#ifdef __cplusplus
}
#endif

}
/***********************************************************************
** End of file
***********************************************************************/

