

#ifndef __SVPWM_H__
#define __SVPWM_H__

#include "main.h"
#include "stm32f1xx_hal.h"



#define PI 3.1416

extern TIM_HandleTypeDef htim1;

typedef struct
{
	uint32_t b_TIM1PrdCnt;
	// b_VrefA = b_Vm * cos(2*pi*b_freq*t)
	// b_VrefB = b_Vm * cos(2*pi*b_freq*t - 2*pi/3)
	// b_VrefC = b_Vm * cos(2*pi*b_freq*t + 2*pi/3)
	float b_VrefA, b_VrefB, b_VrefC;		
	float b_Vm, b_freq;
	float b_VrefAlpha, b_VrefBeta;
	
	uint8_t b_sector;
	float b_VrefAngle;
	
	//space vector
	//0 : 000		1: 001	2: 010	3: 011	4: 100	5: 101	6: 110	7: 111
	uint8_t a_VectorOut[3];
	float a_VectorOutDuty[3];
	uint32_t a_VectorOutDutyCmpr[4];

} BLDC_SVPWMTypeDef;


extern BLDC_SVPWMTypeDef svpwm1;




void Clark_Transformation(float* In_a, 	float* In_b, float* In_c, float* Out_alpha, float* Out_beta);
void InvClark_Transformation(float* In_alpha, 	float* In_beta, float* Out_a, float* Out_b, float* Out_c);
void sectorJudge(BLDC_SVPWMTypeDef* svpwm);
void SpaceVectorUpdate(BLDC_SVPWMTypeDef* svpwm);
void TIM1CmprLoad(void);
void SVPWM1_SpaceVectorDRV(uint8_t spaceVectorOut);
void SVPWM1_SpaceVectorDRV_v2(uint8_t spaceVectorOut);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);




#endif

