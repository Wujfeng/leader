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
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "os/kernel.h"

namespace os {

struct task_params
{
	PCSTR name;								// ��������
	s32	priority;						// �������ȼ�
	u32 stackbase;						// �����ջ����ַ(ע���ֽڶ��룬��ǰϵͳ8�ֽڶ���)
	u32 stacksize;						// �����ջ��С
	void *func;								// ������
	void *parg;								// �����ܲ���
};

class task : public os_object
{
public:
	task(void);
	~task(void);

public:
	struct task_params _params;

public:
	BOOL create(struct task_params *pparams);
	BOOL t_delete(void);
	void sleep(s32 timeouts);
	void msleep(s32 timeoutms);
	void usleep(s32 timeoutus);

	u32 get_cpu_usage(void);

public:
	virtual void run(void *parg) = 0;	// ���麯��

protected:
    static BOOL func(task* ptask);

public:
	inline BOOL set_param(struct task_params *param);
	inline BOOL get_param(struct task_params *param) const;
};

}

/***********************************************************************
** End of file
***********************************************************************/

