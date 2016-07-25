/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/23
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/

#include "transmit.h"

#include "leader_system.h"

namespace app {

transmit::transmit(void)
{
	_params.name = "transmit";
	_params.priority = 0;
	_params.stacksize = 1024;
	_params.func = (void *)thread::func;
	_params.parg = this;
}

transmit::~transmit(void)
{

}

void transmit::run(void *parg)
{
	transmit *p = (transmit *)parg;
	u32 cnt = 0;
	u32 msg_data = 0;
	u32 msg_size = 0;
	msgque *sync_ct = leader_system::get_instance()->get_sync_ct();
	for (cnt = 0; ; cnt++)
	{
		// �ȴ�calc�����͵���Ϣ���У���ȡ���ݰ��������׵�ַ
		//msgque_pend(tran.syncq_calc, &msg_pend, &msg_size, -1);
		//DBG("%s[%d]: pend: msg[0x%08x], size[%d].\n",
		//	tran.ptask->taskname, cnt, msg_pend, msg_size);
		/* TODO:����DMA���䣬���������������ݷ�ͨ����������λ�� */

        sync_ct->pend(&msg_data, &msg_size, 1000);
        msleep(20);
        INF("%s: transmit task is active[%u]...\n", _name, cnt);
		msleep(10);
	}
}

}
/***********************************************************************
** End of file
***********************************************************************/

