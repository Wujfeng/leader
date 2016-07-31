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
	_params.priority = 0;
	_params.stacksize = 1024;
	_params.func = (void *)thread::func;
	_params.parg = this;
}

calculate::~calculate(void)
{

}

void calculate::run(void *parg)
{
	calculate *p = (calculate *)parg;

    u32 cnt = 0;
	msgque *sync_rc = leader_system::get_instance()->get_sync_rc();
	msgque *sync_ct = leader_system::get_instance()->get_sync_ct();

	u32 packet_addr = 0;
    packet *ppacket = NULL;
	packet_attribute_t *pattr = NULL;
	item_mpu_t  *pitem_mpu  = NULL;
	item_magn_t *pitem_magn = NULL;
	item_baro_t *pitem_baro = NULL;
	item_gps_t  *pitem_gps  = NULL;
	item_attitude_t *pitem_attitude = NULL;
	item_rc_t *pitem_rc = NULL;
	for ( ;  ;)
	{
        // �ȴ�rece�����͵���Ϣ���У���ȡ���ݰ��������׵�ַ
		sync_rc->pend(&packet_addr, NULL, 1000);
		ppacket = (packet *)packet_addr;

		pitem_mpu  = (item_mpu_t *)(ppacket->get_item_data(ID_ITEM_MPU));
        core::mdelay(100);
        pitem_mpu->num
        pitem_mpu->data_mpu[0].acce.x
            pitem_mpu->data_mpu[0].acce.y
            pitem_mpu->data_mpu[0].acce.z
            pitem_mpu->data_mpu[0].gyro.z

        pitem_baro = (item_baro_t *)(ppacket->get_item_data(ID_ITEM_BARO));
        pitem_baro->data_baro[0].altitude

        // ������Ϣ���и�tran���񣬽����ݻ������׵�ַ����tran����
		sync_ct->post((void *)packet_addr, sizeof(void *), 1000);

		//DBG("%s: task is active[%u]...\n", _name, cnt++);
	}

}

}
/***********************************************************************
** End of file
***********************************************************************/
