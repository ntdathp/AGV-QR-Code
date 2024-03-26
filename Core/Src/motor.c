#include <math.h>
#include "motor.h"

PROFILE_t tProfile;
Motor_t tMotor1, tMotor2;

void MotorSetRun()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
}

void Motor1Forward()
{
	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
}

void Motor1Backward()
{
	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
}

void Motor2Forward()
{
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

void Motor2Backward()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

void ReadEncoder(Motor_t *tmotor, TIM_HandleTypeDef *htim) {

  tmotor->counter = htim->Instance->CNT;
  htim->Instance->CNT = 0;
  if(tmotor->dir == 1)
  {
	    int16_t temp_data = tmotor->counter;
	    temp_data = - temp_data;
	    tmotor->velocity = (float)temp_data * (SECOND/ SAMPLING_TIME) * tmotor->ptd; // DPS
	    tmotor->position += (float)temp_data * tmotor->ptd; // Degree

  }
  else{
	    int16_t temp_data =  tmotor->counter;
	    tmotor->velocity = (float)temp_data * (SECOND/ SAMPLING_TIME) * tmotor->ptd; // DPS
	    tmotor->position += (float)temp_data * tmotor->ptd; // Degree
  }
}

void MotorSetDuty(uint16_t nDuty, uint8_t channel)
{
  if (channel == MOTOR_1)
  {
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,nDuty);
  }
  else if (channel == MOTOR_2)
  {
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1,nDuty);
  }
}

// init params for timer interrupt...
void MotorInit(void)
{
  HAL_TIM_Base_Start_IT(&htim2);


  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

  PIDInit(&tPID_1, 0.15, 5, 0.001);
  PIDInit(&tPID_2, 0.15, 5, 0.001);
  MotorSetDuty(0, MOTOR_1);
  MotorSetDuty(0, MOTOR_2);

}

void MotorGetPulse(uint32_t *nPulse, uint8_t motor)
{
  if (motor == MOTOR_1)
  {
    *nPulse = __HAL_TIM_GetCounter(&htim4);
  }
  else if (motor == MOTOR_2)
  {
    *nPulse = __HAL_TIM_GetCounter(&htim3);
  }
}

void MotorTrapzoidalInit(PROFILE_t *tProfile, float maxPos, float maxVel, float maxAcc)
{
	if (maxPos <= 0 || maxVel <= 0 || maxAcc <= 0) {

	        return;
	    }
	if (maxVel > sqrt(maxAcc * maxPos)) maxVel = sqrt(maxAcc * maxPos) - 1.0f;
	tProfile->dAccelMax = maxAcc;
	tProfile->dVelMax = maxVel;
	tProfile->dPosMax = maxPos * 0.8725; //Must Increase

	tProfile->dA1 = 0.5f * tProfile->dAccelMax;
	tProfile->dA2 = tProfile->dVelMax;
	tProfile->dB2 = -0.5f * tProfile->dVelMax * tProfile->dVelMax / tProfile->dAccelMax;
	tProfile->dA3 = -0.5f * tProfile->dAccelMax;
	tProfile->dB3 = tProfile->dPosMax * tProfile->dAccelMax / tProfile->dVelMax + tProfile->dVelMax;
	tProfile->dC3 = -0.5f * tProfile->dPosMax * tProfile->dPosMax * tProfile->dAccelMax / (tProfile->dVelMax * tProfile->dVelMax) - 0.5f * tProfile->dVelMax * tProfile->dVelMax / tProfile->dAccelMax;

	tProfile->dMidStep1 = tProfile->dVelMax / tProfile->dAccelMax;
	tProfile->dMidStep2 = tProfile->dPosMax / tProfile->dVelMax;
	tProfile->dMidStep3 = tProfile->dMidStep1 + tProfile->dMidStep2;
	tProfile->nTime = 0;
}
