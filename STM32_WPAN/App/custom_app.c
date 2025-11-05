/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : App/custom_app.c
 * Description        : Custom Example Application (Server)
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wbxx_hal_adc.h"
#include "app_ble.h"
#include "stm32wbxx_hal_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* mySvc */
  uint8_t               Mycharnotify_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Vref_CAL *VREFINT_CAL_ADDR
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */
uint8_t notifyflag = 0;
uint8_t SecureReadData;
extern ADC_HandleTypeDef hadc1;
extern float cell_voltages[5];
extern uint16_t cell_mv[5];
extern uint8_t sleep_flag;
extern uint8_t charge_state;
extern uint16_t last_net_mv;
extern volatile uint16_t capacity;
extern volatile uint32_t nominal_cell_mv_addr;
extern volatile uint32_t charged_cell_mv_addr;
extern volatile uint32_t discharged_cell_mv_addr;
extern volatile uint32_t capacity_addr;
extern volatile uint32_t C_rating_addr;
extern volatile uint32_t type_addr;
extern volatile uint16_t cell_nominal_mv ;
extern volatile uint16_t cell_charged_mv ;
extern volatile uint16_t cell_discharged_mv ;
extern volatile uint16_t lowest_mv;
extern volatile uint8_t type;
extern volatile uint16_t capacity;
extern volatile uint8_t C_rating;
extern volatile uint32_t nickname_addr;
extern volatile uint8_t nickname[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* mySvc */
static void Custom_Mycharnotify_Update_Char(void);
static void Custom_Mycharnotify_Send_Notification(void);

/* USER CODE BEGIN PFP */
int stringToUint8Array(const char* input, uint8_t* UpdateCharData, size_t arraySize) {
    // Check for NULL input or array
    if (input == NULL || UpdateCharData == NULL) {
        return -1; // Error: Invalid input
    }

    // Get length of input string (excluding null terminator)
    size_t len = strlen(input);

    // Check if string fits in array (including null terminator)
    if (len + 1 > arraySize) {
        return -2; // Error: String too long for array
    }

    // Copy string to uint8_t array
    for (size_t i = 0; i < len; i++) {
        UpdateCharData[i] = (uint8_t)input[i];
    }
    UpdateCharData[len] = '\0'; // Add null terminator

    return 0; // Success
}


void myTask(void)
{

	get_voltage();
	  	  UpdateCharData[0] = cell_mv[0]>>8;
		  UpdateCharData[1] = cell_mv[0];
		  UpdateCharData[2] = cell_mv[1]>>8;
		  UpdateCharData[3] = cell_mv[1];
		  UpdateCharData[4] = cell_mv[2]>>8;
		  UpdateCharData[5] = cell_mv[2];
		  UpdateCharData[6] = cell_mv[3]>>8;
		  UpdateCharData[7] = cell_mv[3];
		  UpdateCharData[8] = cell_mv[4]>>8;
		  UpdateCharData[9] = cell_mv[4];
		  UpdateCharData[10] = cell_mv[5]>>8;
		  UpdateCharData[11] = cell_mv[5];

		  uint32_t uid_word0 = HAL_GetUIDw2();

		  UpdateCharData[12] = uid_word0 >> 24;
		  UpdateCharData[13] = uid_word0 >> 16;
		  UpdateCharData[14] = update_state();

		  capacity = read_from_flash(capacity_addr);
		  UpdateCharData[15] = capacity>>8;
		  UpdateCharData[16] = capacity;

		  cell_nominal_mv = read_from_flash(nominal_cell_mv_addr);
		  cell_charged_mv = read_from_flash(charged_cell_mv_addr);
		  cell_discharged_mv = read_from_flash(discharged_cell_mv_addr);

		  UpdateCharData[17] = cell_nominal_mv>>8;
		  UpdateCharData[18] = cell_nominal_mv;
		  UpdateCharData[19] = cell_charged_mv>>8;
		  UpdateCharData[20] = cell_charged_mv;
		  UpdateCharData[21] = cell_discharged_mv>>8;
		  UpdateCharData[22] = cell_discharged_mv;

		  type = (uint8_t)read_from_flash(type_addr);
		  C_rating = (uint8_t)read_from_flash(C_rating_addr);

		  UpdateCharData[23] = type;
		  UpdateCharData[24] = C_rating;

		  Flash_ReadString(nickname_addr,nickname);
		  for(int i =0; i <=16; i++)
		  {
			  UpdateCharData[25+i] =nickname[i];
		  }

		  Custom_Mycharnotify_Update_Char();
if(sleep_flag == 1){
	      }
else
	      {UTIL_SEQ_SetTask(1 << CFG_TASK_MY_TASK, CFG_SCH_PRIO_0);}
}

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* mySvc */
    case CUSTOM_STM_MYCHARWRITE_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MYCHARWRITE_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_MYCHARWRITE_WRITE_EVT */
      break;

    case CUSTOM_STM_MYCHARNOTIFY_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MYCHARNOTIFY_NOTIFY_ENABLED_EVT */
    	notifyflag = 1;
      /* USER CODE END CUSTOM_STM_MYCHARNOTIFY_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_MYCHARNOTIFY_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MYCHARNOTIFY_NOTIFY_DISABLED_EVT */
    	notifyflag = 0;
      /* USER CODE END CUSTOM_STM_MYCHARNOTIFY_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* mySvc */
__USED void Custom_Mycharnotify_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Mycharnotify_UC_1*/
  updateflag = notifyflag;
  /* USER CODE END Mycharnotify_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MYCHARNOTIFY, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Mycharnotify_UC_Last*/

  /* USER CODE END Mycharnotify_UC_Last*/
  return;
}

void Custom_Mycharnotify_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Mycharnotify_NS_1*/
  updateflag = notifyflag;
  /* USER CODE END Mycharnotify_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MYCHARNOTIFY, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Mycharnotify_NS_Last*/

  /* USER CODE END Mycharnotify_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
