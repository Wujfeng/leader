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

#include "calculate.h"

#include "leader_system.h"

namespace app {

calculate::calculate(void)
{
	_params.name = "calculate";
	_params.priority = 5;
	_params.stackbase = NULL;
	_params.stacksize = 512;
	_params.func = (void *)task::func;
	_params.parg = this;
}

calculate::~calculate(void)
{

}

void calculate::run(void *parg)
{
    u32 cnt = 0;
    u32 msg_data = 0;
	u32 msg_size = 0;
    msgque *sync_r_c = leader_system::get_instance()->get_sync_r_c();
    msgque *sync_c_t = leader_system::get_instance()->get_sync_c_t();

	for (cnt = 0;  ;cnt++)
	{
		// �ȴ�rece�����͵���Ϣ���У���ȡ���ݰ��������׵�ַ
		//msgque_pend(calc.syncq_rece, &msg_pend, &msg_size, -1);
		//INF("%s[%d]: pend: msg[0x%08x], size[%d].\n",
		//	calc.ptask->taskname, cnt, msg_pend, msg_size);
		/* TODO:��ʼ���и����������������ں� */


		// ������Ϣ���и�tran���񣬽����ݻ������׵�ַ����tran����
		//msgque_post(calc.syncq_tran, msg_pend, msg_size, -1);
		//INF("%s[%d]: post: msg[0x%08x], size[%d].\n",
		//	calc.ptask->taskname, cnt, msg_pend, msg_size);

        //sync_r_c->pend(&msg_data, &msg_size, 1000);
        //msleep(300);
        //sync_c_t->post(&msg_data, msg_size, 1000);

		INF("%s: task is active[%u]...\n", _params.name, cnt);
		msleep(200);
	}

}

}
/***********************************************************************
** End of file
***********************************************************************/
