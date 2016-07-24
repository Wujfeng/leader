#ifndef  BSP_PRESENT
#define  BSP_PRESENT


#ifdef   BSP_MODULE
#define  BSP_EXT
#else
#define  BSP_EXT  extern
#endif


#include  <stdio.h>
#include  <stdarg.h>
#include  <cpu.h>
#include  <cpu_core.h>
#include  <lib_def.h>
#include  <lib_ascii.h>
#include  "stm32f4xx.h"

#define  BSP_REG_DEM_CR                           (*(CPU_REG32 *)0xE000EDFC)	//DEMCR�Ĵ���
#define  BSP_REG_DWT_CR                           (*(CPU_REG32 *)0xE0001000)  	//DWT���ƼĴ���
#define  BSP_REG_DWT_CYCCNT                       (*(CPU_REG32 *)0xE0001004)	//DWTʱ�Ӽ����Ĵ���
#define  BSP_REG_DBGMCU_CR                        (*(CPU_REG32 *)0xE0042004)

//DEMCR�Ĵ����ĵ�24λ,���Ҫʹ��DWT ETM ITM��TPIU�Ļ�DEMCR�Ĵ����ĵ�24λ��1
#define  BSP_BIT_DEM_CR_TRCENA                    DEF_BIT_24

//DWTCR�Ĵ����ĵ�0λ,��Ϊ1��ʱ��ʹ��CYCCNT������,ʹ��CYCCNT֮ǰӦ���ȳ�ʼ��
#define  BSP_BIT_DWT_CR_CYCCNTENA                 DEF_BIT_00

#endif


