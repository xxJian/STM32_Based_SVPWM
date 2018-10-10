

#include "svpwm.h"
#include "math.h"

#define PI 3.1416

BLDC_SVPWMTypeDef svpwm1;


void Clark_Transformation(float* In_a, 	float* In_b, float* In_c, float* Out_alpha, float* Out_beta)
{
	(*Out_alpha) = (*In_a)*0.6667  - (*In_b)*0.3333 - (*In_c)*0.3333;
	(*Out_beta) = ((*In_b) - (*In_c)) * 0.5774;
}
void InvClark_Transformation(float* In_alpha, 	float* In_beta, float* Out_a, float* Out_b, float* Out_c)
{
	(*Out_a) = (*In_alpha);
	(*Out_b) = -0.5 * (*In_alpha) + 0.8660 * (*In_beta);
	(*Out_c) = -0.5 * (*In_alpha) - 0.8660 * (*In_beta);	
}
void sectorJudge(BLDC_SVPWMTypeDef* svpwm)
{
	svpwm->b_VrefAngle = atan2(svpwm->b_VrefBeta, svpwm->b_VrefAlpha);
	
	if(( svpwm->b_VrefAngle >= 0.0) && ( svpwm->b_VrefAngle < PI/3.0))
	{
		svpwm->b_sector = 1;
	}
	else if(( svpwm->b_VrefAngle >= PI/3.0) && ( svpwm->b_VrefAngle < 2.0*PI/3.0))
	{
		svpwm->b_sector = 2;
	}
	else if(( svpwm->b_VrefAngle >= 2.0*PI/3.0) && ( svpwm->b_VrefAngle < PI))
	{
		svpwm->b_sector = 3;
	}
	else if(( svpwm->b_VrefAngle >= -PI/3.0) && ( svpwm->b_VrefAngle < 0.0))
	{
		svpwm->b_sector = 6;
	}
	else if(( svpwm->b_VrefAngle >= -2.0*PI/3.0) && ( svpwm->b_VrefAngle < -PI/3.0))
	{
		svpwm->b_sector = 5;
	}
	else if(( svpwm->b_VrefAngle >= -PI) && ( svpwm->b_VrefAngle < -2.0*PI/3.0))
	{
		svpwm->b_sector = 4;
	}
	else
	{
		while(1); //computation error
	}
}
int32_t sectorJudge_v2(BLDC_SVPWMTypeDef* svpwm)
{
	float U1,U2,U3;
	int32_t A,B,C,N,b_sector1;
	
	U1 = svpwm->b_VrefBeta;
	U2 = 0.8660*svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.5;
	U3 = -0.8660*svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.5;

	if(U1>0)	{
		A=1;
	}
	else	{
		A=0;
	}
	if(U2>0)	{
		B=1;
	}
	else	{
    		B=0;
	}
	if(U3>0)	{
		C=1;
	}
	else	{
    		C=0;
	}
    
	N = 4*C +2*B +A;
	if( N==3)	{
		b_sector1 = 1;
	}
	else if( N==1){		
    		b_sector1 = 2;
	}
	else if( N==5)	{
    		b_sector1 = 3;
	}
	else if( N==4)	{
    		b_sector1 = 4;
	}
	else if (N==6)	{
    		b_sector1 = 5;
	}
	else if( N==2)	{
    		b_sector1 = 6;
	}	
	return b_sector1;
}
void SpaceVectorUpdate(BLDC_SVPWMTypeDef* svpwm)
{
	switch(svpwm->b_sector)
	{
		case 1:
			svpwm->a_VectorOut[0] = 4;
			svpwm->a_VectorOut[1] = 6;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[1] = svpwm->b_VrefBeta*1.1547;
			svpwm->a_VectorOutDuty[2] = 0;		
			break;
		case 2:
			svpwm->a_VectorOut[0] = 2;
			svpwm->a_VectorOut[1] = 6;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = -svpwm->b_VrefAlpha + svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[1] =  svpwm->b_VrefAlpha + svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[2] = 0;		
			break;
		case 3:				
			svpwm->a_VectorOut[0] = 2;
			svpwm->a_VectorOut[1] = 3;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = svpwm->b_VrefBeta*1.1547;
			svpwm->a_VectorOutDuty[1] = -svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[2] = 0;		
			break;
		case 4:				
			svpwm->a_VectorOut[0] = 1;
			svpwm->a_VectorOut[1] = 3;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = -svpwm->b_VrefBeta*1.1547;
			svpwm->a_VectorOutDuty[1] = -svpwm->b_VrefAlpha + svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[2] = 0;		
			break;
		case 5:
				
			svpwm->a_VectorOut[0] = 1;
			svpwm->a_VectorOut[1] = 5;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = -svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[1] =  svpwm->b_VrefAlpha - svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[2] = 0;		
			break;
		case 6:
				
			svpwm->a_VectorOut[0] = 4;
			svpwm->a_VectorOut[1] = 5;
			svpwm->a_VectorOut[2] = 0;
			
			svpwm->a_VectorOutDuty[0] = svpwm->b_VrefAlpha + svpwm->b_VrefBeta*0.57735;
			svpwm->a_VectorOutDuty[1] = -svpwm->b_VrefBeta*1.1547;
			svpwm->a_VectorOutDuty[2] = 1.0- svpwm->a_VectorOutDuty[0] - svpwm->a_VectorOutDuty[1];		
			break;
		default:
			break;
	}
	
	svpwm->a_VectorOutDutyCmpr[0] = svpwm->a_VectorOutDuty[2] * 14399/2.0;
	svpwm->a_VectorOutDutyCmpr[1] = svpwm->a_VectorOutDuty[0] * 14399/2.0;
	svpwm->a_VectorOutDutyCmpr[2] = svpwm->a_VectorOutDuty[1] * 14399;
	svpwm->a_VectorOutDutyCmpr[3] = svpwm->a_VectorOutDuty[0] * 14399/2.0;
	
	svpwm->a_VectorOutDutyCmpr[1] += svpwm->a_VectorOutDutyCmpr[0];
	svpwm->a_VectorOutDutyCmpr[2] += svpwm->a_VectorOutDutyCmpr[1];
	svpwm->a_VectorOutDutyCmpr[3] += svpwm->a_VectorOutDutyCmpr[2];
	
}
void TIM1CmprLoad(void)
{
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, svpwm1.a_VectorOutDutyCmpr[0]);
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_2, svpwm1.a_VectorOutDutyCmpr[1]);
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_3, svpwm1.a_VectorOutDutyCmpr[2]);
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_4, svpwm1.a_VectorOutDutyCmpr[3]);

}
void SVPWM1_SpaceVectorDRV(uint8_t spaceVectorOut)
{
	if(spaceVectorOut==0)
	{	// 000
		HAL_GPIO_WritePin(BLDC_UH_GPIO_Port, BLDC_UH_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(BLDC_VH_GPIO_Port, BLDC_VH_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(BLDC_WH_GPIO_Port, BLDC_WH_Pin, GPIO_PIN_RESET);	
		
		HAL_GPIO_WritePin(BLDC_UL_GPIO_Port, BLDC_UL_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BLDC_VL_GPIO_Port, BLDC_VL_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(BLDC_WL_GPIO_Port, BLDC_WL_Pin, GPIO_PIN_SET);
	}
	else if(spaceVectorOut==1)
	{	// 001
		HAL_GPIO_WritePin(BLDC_UH_GPIO_Port, BLDC_UH_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(BLDC_VH_GPIO_Port, BLDC_VH_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(BLDC_WL_GPIO_Port, BLDC_WL_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(BLDC_UL_GPIO_Port, BLDC_UL_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BLDC_VL_GPIO_Port, BLDC_VL_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(BLDC_WH_GPIO_Port, BLDC_WH_Pin, GPIO_PIN_SET);	
	}
	else if(spaceVectorOut==2)
	{	// 010
		HAL_GPIO_WritePin(BLDC_UH_GPIO_Port, BLDC_UH_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BLDC_VL_GPIO_Port, BLDC_VL_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(BLDC_WH_GPIO_Port, BLDC_WH_Pin, GPIO_PIN_RESET);	
		
		HAL_GPIO_WritePin(BLDC_UL_GPIO_Port, BLDC_UL_Pin, GPIO_PIN_SET);		
		HAL_GPIO_WritePin(BLDC_VH_GPIO_Port, BLDC_VH_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(BLDC_WL_GPIO_Port, BLDC_WL_Pin, GPIO_PIN_SET);
	}
	else if(spaceVectorOut==3)
	{	// 011
		HAL_GPIO_WritePin(BLDC_UH_GPIO_Port, BLDC_UH_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BLDC_VL_GPIO_Port, BLDC_VL_Pin, GPIO_PIN_RESET);			
		HAL_GPIO_WritePin(BLDC_WL_GPIO_Port, BLDC_WL_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(BLDC_UL_GPIO_Port, BLDC_UL_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BLDC_VH_GPIO_Port, BLDC_VH_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BLDC_WH_GPIO_Port, BLDC_WH_Pin, GPIO_PIN_SET);	
	}
	else if(spaceVectorOut==4)
	{	// 100
		HAL_GPIO_WritePin(BLDC_UL_GPIO_Port, BLDC_UL_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(BLDC_VH_GPIO_Port, BLDC_VH_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(BLDC_WH_GPIO_Port, BLDC_WH_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(BLDC_UH_GPIO_Port, BLDC_UH_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BLDC_VL_GPIO_Port, BLDC_VL_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(BLDC_WL_GPIO_Port, BLDC_WL_Pin, GPIO_PIN_SET);
	}
	else if(spaceVectorOut==5)
	{	// 101
		HAL_GPIO_WritePin(BLDC_UL_GPIO_Port, BLDC_UL_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(BLDC_VH_GPIO_Port, BLDC_VH_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BLDC_WL_GPIO_Port, BLDC_WL_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(BLDC_UH_GPIO_Port, BLDC_UH_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BLDC_VL_GPIO_Port, BLDC_VL_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BLDC_WH_GPIO_Port, BLDC_WH_Pin, GPIO_PIN_SET);		
	}
	else if(spaceVectorOut==6)
	{	// 110
		HAL_GPIO_WritePin(BLDC_UL_GPIO_Port, BLDC_UL_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(BLDC_VL_GPIO_Port, BLDC_VL_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(BLDC_WH_GPIO_Port, BLDC_WH_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(BLDC_UH_GPIO_Port, BLDC_UH_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BLDC_VH_GPIO_Port, BLDC_VH_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(BLDC_WL_GPIO_Port, BLDC_WL_Pin, GPIO_PIN_SET);
	}
	else if(spaceVectorOut==7)
	{	// 111
		HAL_GPIO_WritePin(BLDC_UL_GPIO_Port, BLDC_UL_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(BLDC_VL_GPIO_Port, BLDC_VL_Pin, GPIO_PIN_RESET);			
		HAL_GPIO_WritePin(BLDC_WL_GPIO_Port, BLDC_WL_Pin, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(BLDC_UH_GPIO_Port, BLDC_UH_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BLDC_VH_GPIO_Port, BLDC_VH_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BLDC_WH_GPIO_Port, BLDC_WH_Pin, GPIO_PIN_SET);	
	}
}
void SVPWM1_SpaceVectorDRV_v2(uint8_t spaceVectorOut)
{
	
  uint32_t tmpccmrx = 0U;
	
	
	// 更新本次的空间矢量
	HAL_TIM_GenerateEvent(htim1, TIM_EVENTSOURCE_COM);
	
	
	// 装载下次需要发出的空间矢量
	// CCxE CCxNE OCxM
	// OC1M OC2M in CCMR1			OC3M OC4M in CCMR2
	if(spaceVectorOut==0)
	{	// 000
		//OCxM
		/* Get the TIMx CCMR1 register value */
		tmpccmrx = TIMx->CCMR1;
		/* Reset the Output Compare Mode Bits */
		tmpccmrx &= ~TIM_CCMR1_OC1M;
		tmpccmrx &= ~TIM_CCMR1_OC2M;
		/* Select the Output Compare Mode */
		tmpccmrx |= TIM_OCMODE_INACTIVE;
		tmpccmrx |= (TIM_OCMODE_INACTIVE << 8U);		
			
		/* Write to TIMx CCMR1 */
		htim1.CCMR1 = tmpccmrx;
			
		tmpccmrx = TIMx->CCMR2;
		tmpccmrx &= ~TIM_CCMR2_OC3M;		
		tmpccmrx |= TIM_OCMODE_INACTIVE;
		htim1.CCMR2 = tmpccmrx;
		
		htim1.CCER &= ~TIM_CCER_CC1E;		//CC1E=0
		htim1.CCER |= TIM_CCER_CC1NE;		//CC1NE=1
		htim1.CCER &= ~TIM_CCER_CC2E;		//CC2E=0
		htim1.CCER |= TIM_CCER_CC2NE;		//CC2NE=1
		htim1.CCER &= ~TIM_CCER_CC3E;		//CC3E=0
		htim1.CCER |= TIM_CCER_CC3NE;		//CC3NE=1
	}
	else if(spaceVectorOut==1)
	{	// 001
		//OC1M OC2M
		tmpccmrx = TIMx->CCMR1;
		tmpccmrx &= ~TIM_CCMR1_OC1M;
		tmpccmrx &= ~TIM_CCMR1_OC2M;
		tmpccmrx |= TIM_OCMODE_INACTIVE;
		tmpccmrx |= (TIM_OCMODE_INACTIVE << 8U);		
		htim1.CCMR1 = tmpccmrx;
			
		//OC3M
		tmpccmrx = TIMx->CCMR2;
		tmpccmrx &= ~TIM_CCMR2_OC3M;		
		tmpccmrx |= TIM_OCMODE_ACTIVE;
		htim1.CCMR2 = tmpccmrx;
		
		htim1.CCER &= ~TIM_CCER_CC1E;		//CC1E=0
		htim1.CCER |= TIM_CCER_CC1NE;		//CC1NE=1
		htim1.CCER &= ~TIM_CCER_CC2E;		//CC2E=0
		htim1.CCER |= TIM_CCER_CC2NE;		//CC2NE=1
		htim1.CCER |= TIM_CCER_CC3E;		//CC3E=1
		htim1.CCER &= ~TIM_CCER_CC3NE;		//CC3NE=0
	}
	else if(spaceVectorOut==2)
	{	// 010
		//OC1M OC2M
		tmpccmrx = TIMx->CCMR1;
		tmpccmrx &= ~TIM_CCMR1_OC1M;
		tmpccmrx &= ~TIM_CCMR1_OC2M;
		tmpccmrx |= TIM_OCMODE_INACTIVE;
		tmpccmrx |= (TIM_OCMODE_ACTIVE << 8U);		
		htim1.CCMR1 = tmpccmrx;
			
		//OC3M
		tmpccmrx = TIMx->CCMR2;
		tmpccmrx &= ~TIM_CCMR2_OC3M;		
		tmpccmrx |= TIM_OCMODE_INACTIVE;
		htim1.CCMR2 = tmpccmrx;
		
		htim1.CCER &= ~TIM_CCER_CC1E;		//CC1E	=0
		htim1.CCER |= TIM_CCER_CC1NE;		//CC1NE	=1
		htim1.CCER |= TIM_CCER_CC2E;		//CC2E	=1
		htim1.CCER &= ~TIM_CCER_CC2NE;	//CC2NE	=0
		htim1.CCER &= ~TIM_CCER_CC3E;		//CC3E	=0
		htim1.CCER |= TIM_CCER_CC3NE;		//CC3NE	=1		
	}
	else if(spaceVectorOut==3)
	{	// 011
		//OC1M OC2M
		tmpccmrx = TIMx->CCMR1;
		tmpccmrx &= ~TIM_CCMR1_OC1M;
		tmpccmrx &= ~TIM_CCMR1_OC2M;
		tmpccmrx |= TIM_OCMODE_INACTIVE;
		tmpccmrx |= (TIM_OCMODE_ACTIVE << 8U);		
		htim1.CCMR1 = tmpccmrx;
			
		//OC3M
		tmpccmrx = TIMx->CCMR2;
		tmpccmrx &= ~TIM_CCMR2_OC3M;		
		tmpccmrx |= TIM_OCMODE_ACTIVE;
		htim1.CCMR2 = tmpccmrx;
		
		htim1.CCER &= ~TIM_CCER_CC1E;		//CC1E	=0
		htim1.CCER |= TIM_CCER_CC1NE;		//CC1NE	=1
		htim1.CCER |= TIM_CCER_CC2E;		//CC2E	=1
		htim1.CCER &= ~TIM_CCER_CC2NE;	//CC2NE	=0
		htim1.CCER |= TIM_CCER_CC3E;		//CC3E	=1
		htim1.CCER &= ~TIM_CCER_CC3NE;	//CC3NE	=0
	}
	else if(spaceVectorOut==4)
	{	// 100
		//OC1M OC2M
		tmpccmrx = TIMx->CCMR1;
		tmpccmrx &= ~TIM_CCMR1_OC1M;
		tmpccmrx &= ~TIM_CCMR1_OC2M;
		tmpccmrx |= TIM_OCMODE_ACTIVE;
		tmpccmrx |= (TIM_OCMODE_INACTIVE << 8U);		
		htim1.CCMR1 = tmpccmrx;
			
		//OC3M
		tmpccmrx = TIMx->CCMR2;
		tmpccmrx &= ~TIM_CCMR2_OC3M;		
		tmpccmrx |= TIM_OCMODE_INACTIVE;
		htim1.CCMR2 = tmpccmrx;
		
		htim1.CCER |= TIM_CCER_CC1E;		//CC1E	=1
		htim1.CCER &= ~TIM_CCER_CC1NE;	//CC1NE	=0
		htim1.CCER &= ~TIM_CCER_CC2E;		//CC2E	=0
		htim1.CCER |= TIM_CCER_CC2NE;		//CC2NE	=1
		htim1.CCER &= ~TIM_CCER_CC3E;		//CC3E	=0
		htim1.CCER |= TIM_CCER_CC3NE;		//CC3NE	=1
	}
	else if(spaceVectorOut==5)
	{	// 101
		//OC1M OC2M
		tmpccmrx = TIMx->CCMR1;
		tmpccmrx &= ~TIM_CCMR1_OC1M;
		tmpccmrx &= ~TIM_CCMR1_OC2M;
		tmpccmrx |= TIM_OCMODE_ACTIVE;
		tmpccmrx |= (TIM_OCMODE_INACTIVE << 8U);		
		htim1.CCMR1 = tmpccmrx;
			
		//OC3M
		tmpccmrx = TIMx->CCMR2;
		tmpccmrx &= ~TIM_CCMR2_OC3M;		
		tmpccmrx |= TIM_OCMODE_ACTIVE;
		htim1.CCMR2 = tmpccmrx;
		
		htim1.CCER |= TIM_CCER_CC1E;		//CC1E	=1
		htim1.CCER &= ~TIM_CCER_CC1NE;	//CC1NE	=0
		htim1.CCER &= ~TIM_CCER_CC2E;		//CC2E	=0
		htim1.CCER |= TIM_CCER_CC2NE;		//CC2NE	=1
		htim1.CCER |= TIM_CCER_CC3E;		//CC3E	=1
		htim1.CCER &= ~TIM_CCER_CC3NE;	//CC3NE	=0
	}
	else if(spaceVectorOut==6)
	{	// 110
		//OC1M OC2M
		tmpccmrx = TIMx->CCMR1;
		tmpccmrx &= ~TIM_CCMR1_OC1M;
		tmpccmrx &= ~TIM_CCMR1_OC2M;
		tmpccmrx |= TIM_OCMODE_ACTIVE;
		tmpccmrx |= (TIM_OCMODE_ACTIVE << 8U);		
		htim1.CCMR1 = tmpccmrx;
			
		//OC3M
		tmpccmrx = TIMx->CCMR2;
		tmpccmrx &= ~TIM_CCMR2_OC3M;		
		tmpccmrx |= TIM_OCMODE_INACTIVE;
		htim1.CCMR2 = tmpccmrx;
		
		htim1.CCER |= TIM_CCER_CC1E;		//CC1E	=1
		htim1.CCER &= ~TIM_CCER_CC1NE;	//CC1NE	=0
		htim1.CCER |= TIM_CCER_CC2E;		//CC2E	=1
		htim1.CCER &= ~TIM_CCER_CC2NE;	//CC2NE	=0
		htim1.CCER &= ~TIM_CCER_CC3E;		//CC3E	=0
		htim1.CCER |= TIM_CCER_CC3NE;		//CC3NE	=1
	}
	else if(spaceVectorOut==7)
	{	// 111
		//OC1M OC2M
		tmpccmrx = TIMx->CCMR1;
		tmpccmrx &= ~TIM_CCMR1_OC1M;
		tmpccmrx &= ~TIM_CCMR1_OC2M;
		tmpccmrx |= TIM_OCMODE_ACTIVE;
		tmpccmrx |= (TIM_OCMODE_ACTIVE << 8U);		
		htim1.CCMR1 = tmpccmrx;
			
		//OC3M
		tmpccmrx = TIMx->CCMR2;
		tmpccmrx &= ~TIM_CCMR2_OC3M;		
		tmpccmrx |= TIM_OCMODE_ACTIVE;
		htim1.CCMR2 = tmpccmrx;
		
		htim1.CCER |= TIM_CCER_CC1E;		//CC1E	=1
		htim1.CCER &= ~TIM_CCER_CC1NE;	//CC1NE	=0
		htim1.CCER |= TIM_CCER_CC2E;		//CC2E	=1
		htim1.CCER &= ~TIM_CCER_CC2NE;	//CC2NE	=0
		htim1.CCER |= TIM_CCER_CC3E;		//CC3E	=1
		htim1.CCER &= ~TIM_CCER_CC3NE;	//CC3NE	=0
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		svpwm1.b_VrefA  = svpwm1.Vm * cos(6.2832*svpwm1.b_freq*svpwm1.b_TIM1PrdCnt/5000.0);
		svpwm1.b_VrefB  = svpwm1.Vm * cos(6.2832*svpwm1.b_freq*svpwm1.b_TIM1PrdCnt/5000.0 - 6.2832/3.0);
		svpwm1.b_VrefC  = svpwm1.Vm * cos(6.2832*svpwm1.b_freq*svpwm1.b_TIM1PrdCnt/5000.0 + 6.2832/3.0);
		
		// 0 <= Vm <=1
		// to avoid over modulation, scale down the Vref
		svpwm1.b_VrefA *= 0.8660;
		svpwm1.b_VrefB *= 0.8660;
		svpwm1.b_VrefC *= 0.8660;
		
		Clark_Transformation(&svpwm1.b_VrefA, &svpwm1.b_VrefB, &svpwm1.b_VrefC, &svpwm1.b_VrefAlpha, &svpwm1.b_VrefBeta);
		sectorJudge(&svpwm1);
		SpaceVectorUpdate(&svpwm1);
		
		TIM1CmprLoad();
		
		SVPWM1_SpaceVectorDRV(svpwm1.a_VectorOut[2]);
		
		svpwm1.b_TIM1PrdCnt++;
		if(svpwm1.b_TIM1PrdCnt==100)
		{
			svpwm1.b_TIM1PrdCnt = 0;
		}
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		SVPWM1_SpaceVectorDRV(svpwm1.a_VectorOut[0]);		
	}
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		SVPWM1_SpaceVectorDRV(svpwm1.a_VectorOut[1]);				
	}
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		SVPWM1_SpaceVectorDRV(svpwm1.a_VectorOut[0]);			
	}
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		SVPWM1_SpaceVectorDRV(svpwm1.a_VectorOut[2]);			
	}

}
