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
#include "receive.h"

#include "leader_system.h"

namespace app {

receive::receive(void)
{
	_params.name = "receive";
	_params.priority = 0;
	_params.stacksize = 512;
	_params.func = (void *)thread::func;
	_params.parg = this;
}

receive::~receive(void)
{

}

void receive::run(void *parg)
{
  	receive *p = (receive *)parg;

	msgque *sync_rc = leader_system::get_instance()->get_sync_rc();

	mpu6000 *mpu6000 = leader_system::get_instance()->get_mpu6000();

	packet *ppacket  = leader_system::get_instance()->get_packet();
	u32 packet_addr = (u32)ppacket;
	packet_attribute_t *pattr = ppacket->get_attribute();
	item_mpu_t  *pitem_mpu  = (item_mpu_t *)(ppacket->get_item_data(ID_ITEM_MPU));
	item_magn_t *pitem_magn = (item_magn_t *)(ppacket->get_item_data(ID_ITEM_MAGN));
	item_baro_t *pitem_baro = (item_baro_t *)(ppacket->get_item_data(ID_ITEM_BARO));
	item_gps_t  *pitem_gps  = (item_gps_t *)(ppacket->get_item_data(ID_ITEM_GPS));
	item_attitude_t *pitem_atti = (item_attitude_t *)(ppacket->get_item_data(ID_ITEM_ATTITUDE));
	item_rc_t *pitem_rc = (item_rc_t *)ppacket->get_item_data(ID_ITEM_RC);
	
	for (u32 cnt = 0; ;cnt++)
	{
		for (u32 i = 0; i < pattr->num_mpu; i++)
		{
			pitem_mpu->data_mpu[i].acce.x = 1;
			pitem_mpu->data_mpu[i].acce.y = 2;
			pitem_mpu->data_mpu[i].acce.z = 3;
			pitem_mpu->data_mpu[i].gyro.x = 4;
			pitem_mpu->data_mpu[i].gyro.y = 5;
			pitem_mpu->data_mpu[i].gyro.z = 6;
		}

		for (u32 i = 0; i < pattr->num_magn; i++)
		{
			pitem_magn->data_magn[i].x = 1;
			pitem_magn->data_magn[i].y = 2;
			pitem_magn->data_magn[i].z = 3;
		}

		for (u32 i = 0; i < pattr->num_attitude; i++)
		{
			pitem_atti->data_attitude[i].roll= 1;
			pitem_atti->data_attitude[i].pitch= 2;
			pitem_atti->data_attitude[i].yaw = 3;
			pitem_atti->data_attitude[i].altitude = 4;
		}

		for (u32 i = 0; i < pattr->num_rc; i++)
		{
			pitem_rc->data_rc[i].throttle= 1;
			pitem_rc->data_rc[i].roll= 2;
			pitem_rc->data_rc[i].pitch= 3;
			pitem_rc->data_rc[i].yaw = 4;
			pitem_rc->data_rc[i].aux1 = 1;
			pitem_rc->data_rc[i].aux2 = 2;
			pitem_rc->data_rc[i].aux3 = 3;
			pitem_rc->data_rc[i].aux4 = 4;
			pitem_rc->data_rc[i].aux5 = 5;
			pitem_rc->data_rc[i].aux5 = 6;
		}
		
		sync_rc->post((void *)packet_addr, sizeof(void *), 1000);
        
		INF("%s: task is active[%u]...\n", _name, cnt);
		msleep(500);
	}








	
	#if 0
	Mpu6000 *pmpu6000 = SystemUav::get_system_uav()->get_mpu6000();
    Packet  *ppacket  = SystemUav::get_system_uav()->get_packet();
    packet_attribute_t *pattr = &(ppacket->m_attr);

    item_mpu_t  *pitem_mpu  = (item_mpu_t *)(ppacket->get_item_data(ID_ITEM_MPU));
    item_magn_t *pitem_magn = (item_magn_t *)(ppacket->get_item_data(ID_ITEM_MAGN));
    item_baro_t *pitem_baro = (item_baro_t *)(ppacket->get_item_data(ID_ITEM_BARO));
    item_gps_t  *pitem_gps  = (item_gps_t *)(ppacket->get_item_data(ID_ITEM_GPS));
    item_attitude_t *pitem_attitude = (item_attitude_t *)(ppacket->get_item_data(ID_ITEM_ATTITUDE));

    data_mpu_t *data_mpu_buf = pitem_mpu->data_mpu;
    uint32_t data_mpu_size = pattr->num_raw_mpu * sizeof(*(pitem_mpu->data_mpu));

	for (uint32_t cnt = 0, index = 0; ;)
	{
		/* TODO: �����������򴫸������ݶ�ȡ����������DMA���� */
		// ��ȡmpu����������
		// ������Ϣ���и�calc���񣬽����ݻ������׵�ַ����calc����
		//msgque_post(rece.syncq_calc, &packet[index], msg_size, -1);
		//DBG("%s[%d]: post: msg[0x%08x], size[%d].\n",
		//	rece.ptask->taskname, cnt, &packet[index], msg_size);

		// Э�������������ʺ͵������Ƶ��
		//task_sleep(500);

		INF("active: %s.\n", m_param.name);
        pmpu6000->read((int8_t *)data_mpu_buf, data_mpu_size);

        for (uint32_t i = 0; i < pattr->num_raw_mpu; i++)
        {
            INF("\rax=%6.5d, ay=%6.5d, az=%6.5d, gx=%6.5d, gy=%6.5d, gz=%6.5d.",
    		    data_mpu_buf[i].acce_x,
    		    data_mpu_buf[i].acce_y,
    		    data_mpu_buf[i].acce_z,
    		    data_mpu_buf[i].gyro_x,
    		    data_mpu_buf[i].gyro_y,
    		    data_mpu_buf[i].gyro_z
		        );
        }

       // m_pmsgque->post(ppacket, NULL, WAIT_FOREVER);
        msleep(300);
	}
    #endif
}

}
/***********************************************************************
** End of file
***********************************************************************/

