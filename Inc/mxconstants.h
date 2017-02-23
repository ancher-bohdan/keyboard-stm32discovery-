/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Column3_Pin GPIO_PIN_14
#define Column3_GPIO_Port GPIOB
#define RowTop_Pin GPIO_PIN_15
#define RowTop_GPIO_Port GPIOB
#define Column1_Pin GPIO_PIN_12
#define Column1_GPIO_Port GPIOD
#define RowBottom_Pin GPIO_PIN_13
#define RowBottom_GPIO_Port GPIOD
#define Row3_Pin GPIO_PIN_8
#define Row3_GPIO_Port GPIOC
#define Row2_Pin GPIO_PIN_9
#define Row2_GPIO_Port GPIOC
#define Column2_Pin GPIO_PIN_8
#define Column2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define KEYBOARD_STAR					10
#define KEYBOARD_SHARP				11

/* How many digits may appear in room number. Define the length of control queue */
#define ROOM_NUMBER_LENGTH		4

/* The length of key. Either RFID or iButton */
#define KEY_LENGTH						6

/* This value will be added to the queue, if we got a key */
#define KEY_FLAG							0xFF

/* This code we send to ATiny in case of wrong key */
#define KEY_ERROR							0xAA

/* Auto reload register value */
#define ARR_value							5


#define FIRST_LED_ON			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
#define SECOND_LED_ON			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
#define THIRD_LED_ON			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
#define FOUTH_LED_ON			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

#define FIRST_LED_OFF			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
#define SECOND_LED_OFF		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
#define THIRD_LED_OFF			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
#define FOUTH_LED_OFF			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

#define ALL_LED_ON				FIRST_LED_ON \
													SECOND_LED_ON \
													THIRD_LED_ON \
													FOUTH_LED_ON
													
#define ALL_LED_OFF				FIRST_LED_OFF \
													SECOND_LED_OFF \
													THIRD_LED_OFF \
													FOUTH_LED_OFF

typedef enum _error_key_code {ERROR_OK = 1, ERROR_WRONG = 0} error_key_code;
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
