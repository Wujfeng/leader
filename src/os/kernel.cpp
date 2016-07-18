/*******************************Copyright (c)***************************
** 
** Porject name:	LeaderUAV-Plus
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "os/kernel.h"

#include "driver/core.h"
#include "includes.h"
#include "os_cfg_app.h"
/*
CPU_CFG_CRITICAL_METHOD

���ƽ����ٽ�������ģʽ
OS_CFG_ISR_POST_DEFERRED_EN ��os_cfg.h�ļ��ж���
0:ʹ��ֱ�ӷ���ģʽ:��ֹ�ж�
	���ٽ�δ�����ùر��жϵı�����ʩ�������ͻ��ӳ��жϵ���Ӧʱ��
1:ʹ���ӳٷ���ģʽ:��ֹ����
	����ͨ�����жϣ�����ͨ������������������ķ����������ٽ�δ��룬
	���ӳٷ���ģʽ�»��������ڹر��жϵ����

//�����ٽ������˳��ٽ���
#define  OS_CRITICAL_ENTER()                    CPU_CRITICAL_ENTER()
#define  OS_CRITICAL_EXIT()                     CPU_CRITICAL_EXIT()
#define  OS_CRITICAL_EXIT_NO_SCHED()            CPU_CRITICAL_EXIT()
*/

#define INIT_CRITICAL_SECTION() 		CPU_SR_ALLOC()		// ��ʼ���ٽ���
#define ENTER_CRITICAL_SECTION()		OS_CRITICAL_ENTER() // �����ٽ���
#define EXIT_CRITICAL_SECTION() 		OS_CRITICAL_EXIT()	// �˳��ٽ���


/**
 * ϵͳTick�жϷ�����
 */
extern "C" void SysTick_Handler(void)
{
	OSIntEnter();							// �����ж�
    OSTimeTick();       					// ����ucos��ʱ�ӷ������
    OSIntExit();        					// ���������л����ж�
}



namespace os {

kernel::kernel(void)
{

}

kernel::~kernel(void)
{

}

/**
 *  ��ʼ������ϵͳTick
 */
void kernel::systick_init(void)
{
	/*
	 * ����OS_TICKS_PER_SEC�趨���ʱ��
	 * reloadΪ24λ�Ĵ���,���ֵ:16777216,��168M��,Լ��0.7989s����
	 */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	u32 sysclk = driver::core::get_cpu_freq();
	u32 reload = sysclk / 8 / OS_CFG_TICK_RATE_HZ;
#if 0
	if (SysTick_Config(reload))
	{
		/* Capture error */
		while (1);
	}
#elif 1
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; 	// ����SYSTICK�ж�
	/* TODO: OS_CFG_TICK_RATE_HZ ֱ�Ӹ���reload/2�����ܷ�����tick�������� */
	SysTick->LOAD = reload;					 	// ÿ1/OS_CFG_TICK_RATE_HZ���ж�һ��
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  	// ����SYSTICK
#else
	OS_CPU_SysTickInit(OS_CFG_TICK_RATE_HZ);
#endif
}

/**
 * ��ʼ������ϵͳ
 * note �ڵ��ò���ϵͳ���κ�API֮ǰ��ʼ������ϵͳ
 */
void kernel::init(void)
{
    s32 error = 0;

    kernel::systick_init();

	// ��ʼ��UCOSIII
	OSInit((OS_ERR *)&error);
	if (error != OS_ERR_NONE)
	{
		ERR("error: %s failed, error = %d.\n", __FUNCTION__, error);
	}
}

/**
 *  ��������ϵͳ
 */
void kernel::start(void)
{
	s32 error = 0;
	// ����UCOSIII
	OSStart((OS_ERR *)&error);
	if (error != OS_ERR_NONE)
	{
		ERR("error: %s failed, error = %d.\n", __FUNCTION__, error);
	}
}

/**
 *  �˳�����ϵͳ
 */
void kernel::exit(void)
{

}

/**
 *  ?????CPU???
 *  return ????CPU???,??????(%)
 *  note   ???? OS_StatTask()
 */
u32 kernel::get_cpu_load(void)
{
	return 0;
}

/**
 *  ???????Tick???
 *  return ????Tick??,???us
 */
u32 kernel::get_tick_period(void)
{
    return (1 * 1000000 / OS_CFG_TICK_RATE_HZ);
}

/**
 *  ??????ms??????????Tick??
 *  ms	   ?????????,??(ms)
 *  return ????????Tick?,??(tick)
 *  note:?ms?-1?,????OSTimeDly???????
 */
u32 kernel::convertmstotick(s32 ms)
{
	/**
	 * ucosiii???:
	 * 1.??OSTimeDly,tick=0?????????(No delay will result if the specified delay is 0.)
	 * 2.?????????????,?????,tick=0???????
	 *   (wait forever until the resource becomes available or the event occurs.)
	 */
	u32 tick = 0;
	u32 period = get_tick_period();
	if (ms < 0)			// ????
	{
		tick = 0;
	}
	else if (ms == 0)	// ???
	{
		tick = 1;
	}
	else				// ??ms?????
	{
		//?????:tick = ms * 1000 / period;
		//tick = CEIL_DIV(ms * 1000, period);
		tick = (ms * 1000 + period - 1) / period;
	}

	return tick;
}

void kernel::on_error(enum os_error_code errcode, os_object* pobject)
{
	static const char* error_message_tab[] =
	{
		"err_no",
		"err_unknown",
		"err_invalid_handle",
		"err_invalid_param",
		"err_out_memory",
		"err_operation_timeout",
		"err_magque_full",
		"err_operation_failed",
		"err_operation_unsupported",
		"err_application",
	};

	// Default error handler
	ERR("OS ERROR: %s [Code=0x%08x, Object=0x%08x, Name=%s, Handle=0x%08x].\n",
		errcode < ARRAYSIZE(error_message_tab) ? error_message_tab[errcode] : "",
		errcode,
		pobject,
		pobject ? pobject->get_name() : "<NULL>",
		pobject ? pobject->get_handle() : 0
		);
}

}
/***********************************************************************
** End of file
***********************************************************************/