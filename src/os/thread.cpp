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
#include "os/thread.h"

#include "includes.h"

namespace os {

thread::thread(void)
{

}

thread::~thread(void)
{

}

BOOL thread::create(struct thread_params *pparams)
{
    s32 error = OS_ERR_NONE;
    if (pparams != NULL) {
        _params = *pparams;
    }

    /*
     * �������ȼ��û�����ʹ��
     * ���ȼ�0���жϷ������������� OS_IntQTask()
     * ���ȼ�1��ʱ�ӽ������� OS_TickTask()
     * ���ȼ�2����ʱ���� OS_TmrTask()
     * ���ȼ�OS_CFG_PRIO_MAX-2��ͳ������ OS_StatTask()
     * ���ȼ�OS_CFG_PRIO_MAX-1���������� OS_IdleTask()
     */
    if (_params.priority > OS_CFG_PRIO_MAX - 3) {
        _params.priority = OS_CFG_PRIO_MAX - 3;
    }
    else if (_params.priority < 3) {
        _params.priority = 3;
    }

    if (_params.stacksize < OS_CFG_STK_SIZE_MIN) {
        _params.stacksize = OS_CFG_STK_SIZE_MIN;
    }

    _name   = _params.name;
    _handle = (HANDLE)malloc(sizeof(OS_TCB));
    //_handle = (HANDLE)new char[sizeof(OS_TCB)];
    if (_handle == NULL) {
        goto fail0;
    }
    /* TODO: ��̬���������ջ */
    _params.stackbase = (u32)malloc(_params.stacksize * 4);
    if (_params.stackbase == NULL) {
        goto fail1;
    }

    /*
     * ��������:UCOSIIIҪ�����񴴽��ڼ��������ٽ���
     */
    CPU_SR_ALLOC();
    OS_CRITICAL_ENTER();        // �����ٽ���
    OSTaskCreate(
        (OS_TCB *)      (_handle),                 // ������ƿ�
        (CPU_CHAR *)    (_params.name),             // ��������
        (OS_TASK_PTR)   (_params.func),              // ������
        (void *)        (_params.parg),             // ���ݸ��������Ĳ���
        (OS_PRIO)       (_params.priority),         // �������ȼ�
        (CPU_STK *)     (_params.stackbase),        // �����ջ����ַ
        (CPU_STK_SIZE)  ((_params.stacksize)/10),   // �����ջ�����λ
        (CPU_STK_SIZE)  (_params.stacksize),        // �����ջ��С
        (OS_MSG_QTY)    0,                          // �����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
        (OS_TICK)       0,                          // ��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
        (void *)        0,                          // �û�����Ĵ洢��
        (OS_OPT)        (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR), // ����ѡ��
        (OS_ERR *)      &error                      // ��Ÿú�������ʱ�ķ���ֵ(0:��ȷ���أ�<0:������)
        );
    OS_CRITICAL_EXIT();         // �˳��ٽ���

    if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
        goto fail2;
    }

    DBG("OSTaskCreate: %s\n", _params.name);
    return true;

fail2:
    free((void *)_params.stackbase);
    _params.stackbase = NULL;
fail1:
    free((void *)_handle);
    _handle = NULL;
fail0:
    return false;
}

BOOL thread::t_delete(void)
{
	s32 error = OS_ERR_NONE;

	//OSTaskDel(0, (OS_ERR *)&error);
	OSTaskDel((OS_TCB *)(_handle), (OS_ERR *)&error);
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}

	free((void *)_params.stackbase);
    _params.stackbase = NULL;

	free((void *)_handle);
    _handle = NULL;

	return true;
}

/**
 *  ʹ��ǰ����˯��ָ��ʱ��(����ȷ�ӳ٣���ȷ�ӳ�ʹ�ö�ʱ���ж�POST�ź���)
 *  timeoutms ˯��ʱ�䣨��λ��ms��
 *  note 1.��timeoutms>=0ʱ��ʹ��ǰ����˯��timeoutmsʱ�䣬ֱ����ʱ�򱻻���
 *		 2.��timeoutms<0ʱ��ʹ��ǰ��������˯��(���˯��0xffffffff tich)��ֱ��������
 */
void thread::msleep(s32 timeoutms)
{
    s32 error = OS_ERR_NONE;
	u32 tick = 0;
	if (timeoutms < 0) {
		// note:��msΪ-1ʱ����������OSTimeDly���ӳٲ���ת��
		tick = 0xffffffff;
		//OSTimeDlyHMSM(99, 59, 59, 999, OS_OPT_TIME_PERIODIC/*OS_OPT_TIME_HMSM_STRICT*/, (OS_ERR *)&error);
	}
	else {
		tick = kernel::convertmstotick(timeoutms);
		//OSTimeDlyHMSM(0, 0, 0, timeoutms, OS_OPT_TIME_PERIODIC/*OS_OPT_TIME_HMSM_STRICT*/, (OS_ERR *)&error);
	}
	OSTimeDly (tick, OS_OPT_TIME_PERIODIC, (OS_ERR *)&error);
	if (error != OS_ERR_NONE) {
        DBG("%s error code = %d.\n", __FUNCTION__, error);
        kernel::on_error(ERR_OPERATION_FAILED, this);
    }

}

BOOL thread::func(thread* pthread)
{
	INF("START======================>\n");
	INF("name: %s.\n",          pthread->_params.name);
	INF("priority: %d.\n",      pthread->_params.priority);
	INF("stackbase: 0x%08x.\n", pthread->_params.stackbase);
	INF("stacksize: 0x%08x.\n", pthread->_params.stacksize);
	INF("END<========================\n");

	pthread->run(pthread->_params.parg);

	return true;
}

void thread::run(void *parg)
{
	ERR("Pure virtual function called: %s(...).\n", __FUNCTION__);
	ASSERT(0);
}


BOOL thread::set_param(struct thread_params *param)
{
    if (param == NULL) return false;
    _params = *param;
    return true;
}

BOOL thread::get_param(struct thread_params *param) const
{
    if (param == NULL) return false;
    *param = _params;
    return true;
}

}

/***********************************************************************
** End of file
***********************************************************************/

