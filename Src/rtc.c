/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
#if (RTC_SET_VALUES==1)
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */
#endif
//extern FLASH_DATA_ORG FlashDataOrg;
//hrtc.Init.AsynchPrediv = *((uint32_t*)(DATA_EEPROM_BASE + FlashDataOrg.b_status.s8_offset));
  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  RtcSynchPrediv = RTC_SYNCH_PREDIV_LSE;
#if (RTC_SET_VALUES==1)
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x57;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x21;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable Calibrartion
  */
  if (HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_512HZ) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
#else
  RTC_AlarmTypeDef sAlarm = {0};
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable Calibration
  */
  if (HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_512HZ) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*************************************************************************
 *                          MY RTC CODE
 ************************************************************************/
//#pragma GCC optimize ("O0")
void CheckDayLigth(RTC_HandleTypeDef* rtcHandle, uint8_t sec, uint8_t min, uint8_t hour,
												 uint8_t day, uint8_t month, uint8_t year)
{
	uint8_t Index;
	uint32_t Counter, DayLigthPeriod;
	int32_t OffSet;
	const uint8_t StartYear = 23;
	/*
	 * Epoch of the entry into force of daylight saving time and the return to solar time in the years
	 * 2023..2035, keeping in mind that the system start date is 01/01/2000 (Unix epoch time: 946684800)
	 */
	const uint32_t EpochDayLightValues[13][2] =
	{
		{733111200,   751863600},	/* 26/03/2023 02:00:00, 29/10/2023 03:00:00 - initializers for row indexed by  0 */
		{765165600,   783313200},	/* 31/03/2024 02:00:00, 27/10/2024 03:00:00 - initializers for row indexed by  1 */
		{796615200,   814762800},	/* 30/03/2025 02:00:00, 26/10/2025 03:00:00 - initializers for row indexed by  2 */
		{828064800,   846212400},	/* 29/03/2026 02:00:00, 25/10/2026 03:00:00 - initializers for row indexed by  3 */
		{859514400,   878266800},	/* 28/03/2027 02:00:00, 31/10/2027 03:00:00 - initializers for row indexed by  4 */
		{890964000,   909716400},	/* 26/03/2028 02:00:00, 29/10/2028 03:00:00 - initializers for row indexed by  5 */
		{922413600,   941166000},	/* 25/03/2029 02:00:00, 28/10/2029 03:00:00 - initializers for row indexed by  6 */
		{954468000,   972615600},	/* 31/03/2030 02:00:00, 27/10/2030 03:00:00 - initializers for row indexed by  7 */
		{985917600,  1004065200},	/* 30/03/2031 02:00:00, 26/10/2031 03:00:00 - initializers for row indexed by  8 */
		{1017367200, 1036119600},	/* 28/03/2032 02:00:00, 31/10/2032 03:00:00 - initializers for row indexed by  9 */
		{1048816800, 1067569200},	/* 27/03/2033 02:00:00, 30/10/2033 03:00:00 - initializers for row indexed by 10 */
		{1080266400, 1099018800},	/* 26/03/2034 02:00:00, 29/10/2034 03:00:00 - initializers for row indexed by 11 */
		{1111716000, 1130468400} 	/* 25/03/2035 02:00:00, 28/10/2035 03:00:00 - initializers for row indexed by 12 */
	};

	Counter = RTC_ToEpoch(sec, min, hour, day, month, year);
//	Counter = RTC_GetCounter(rtcHandle);
	DayLigthPeriod = HAL_RTC_DST_ReadStoreOperation(rtcHandle);
	Index = year - StartYear;
	if ((Index < 0) || (Index > 12))
		return;
	/*
	 * When switching back to standard time, one hour is subtracted from the RTC Counter value.
	 * This causes a bounce between the daylight saving time setting and the standard time setting in the comparison on line 255.
     * To avoid this bounce, when switching back to standard time, an offset of one hour is added to the second member of the
     * comparison on line 255.
     * The offset value returns to zero when more than one hour has passed since switching back to standard time
     * and when daylight saving time is in effect.
	 */
	if (!DayLigthPeriod)
	{
		OffSet = Counter - (EpochDayLightValues[Index][1] - 3600);
		if ((OffSet < 0) || (OffSet > 3600))
		{
			OffSet = 0;
		} else
		if ((OffSet >= 0) && (OffSet <= 3600))
		{
			OffSet = 3600;
		}
	} else
	{
		OffSet = 0;
	}

	if ((Counter > EpochDayLightValues[Index][0]) && ((Counter + (uint32_t)OffSet) < EpochDayLightValues[Index][1]))
	{
		if (!DayLigthPeriod)
		{
			HAL_RTC_DST_Add1Hour(rtcHandle);
			HAL_RTC_DST_SetStoreOperation(rtcHandle);
		}
	} else
	{
		if (DayLigthPeriod)
		{
			HAL_RTC_DST_Sub1Hour(rtcHandle);
			HAL_RTC_DST_ClearStoreOperation(rtcHandle);
		}
	}
}

/**
  * @brief  Gets the RTC counter value.
  * @param  None
  * @retval RTC counter value.
  */
uint32_t RTC_GetCounter(RTC_HandleTypeDef* rtcHandle)
{
	return ((uint32_t)(rtcHandle->Instance->TR & RTC_TR_RESERVED_MASK));
}

/**
 * @brief  Handles the data timestamp
 * @param  Stamp the stamp (time + date)
 * @retval None
 */
void RTC_DateTimeStamp(RTC_HandleTypeDef* rtcHandle, DateTime_t *Stamp)
{
	RTC_DateTypeDef date;
	RTC_TimeTypeDef time;

	if (HAL_RTC_GetTime(rtcHandle, &time, FORMAT_BIN) != HAL_OK)
	{
		HAL_RTC_MspDeInit(rtcHandle);
		HAL_RTC_MspInit(rtcHandle);
	}
	if (HAL_RTC_GetDate(rtcHandle, &date, FORMAT_BIN) != HAL_OK)
	{
		HAL_RTC_MspDeInit(rtcHandle);
		HAL_RTC_MspInit(rtcHandle);
	}

#if (RTC_SET_VALUES==0)
	CheckDayLigth(rtcHandle, time.Seconds, time.Minutes, time.Hours, date.Date, date.Month, date.Year);
#endif

	Stamp->date[0] = (uint8_t)date.Month;
	Stamp->date[1] = (uint8_t)date.Date;
	Stamp->date[2] = (uint8_t)date.Year;
	Stamp->time[0] = (uint8_t)time.Hours;
	Stamp->time[1] = (uint8_t)time.Minutes;
	Stamp->time[2] = (uint8_t)time.Seconds;
}

/**
 * @brief  Handles the time+date getting/sending
 * @param  Msg the time+date part of the stream
 * @retval None
 */
void RTC_Handler(RTC_HandleTypeDef* rtcHandle, uint8_t* Buff)
{
	uint8_t sub_sec = 0;
	uint32_t ans_uint32;
	int32_t ans_int32;
	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructure;

	if(rtcHandle->Instance==RTC)
	{
		if (HAL_RTC_GetTime(rtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
		{
			HAL_RTC_MspDeInit(rtcHandle);
			HAL_RTC_MspInit(rtcHandle);
		}
		if (HAL_RTC_GetDate(rtcHandle, &sdatestructureget, FORMAT_BIN) != HAL_OK)
		{
			HAL_RTC_MspDeInit(rtcHandle);
			HAL_RTC_MspInit(rtcHandle);
		}

		/* To be MISRA C-2012 compliant the original calculation:
		sub_sec = ((((((int)RtcSynchPrediv) - ((int)stimestructure.SubSeconds)) * 100) / (RtcSynchPrediv + 1)) & 0xFF);
		has been split to separate expressions */
		ans_int32 = (RtcSynchPrediv - (int32_t)stimestructure.SubSeconds) * 100;
		ans_int32 /= RtcSynchPrediv + 1;
		ans_uint32 = (uint32_t)ans_int32 & 0xFFU;
		sub_sec = (uint8_t)ans_uint32;

		Buff[3] = (uint8_t)stimestructure.Hours;
		Buff[4] = (uint8_t)stimestructure.Minutes;
		Buff[5] = (uint8_t)stimestructure.Seconds;
		Buff[6] = sub_sec;

		if (!(Buff[3] | Buff[4]))
		{
			MidNight = true;
		} else
		{
			MidNight = false;
		}
	}
}

/**
 * @brief  Configures the current date
 * @param  y the year value to be set
 * @param  m the month value to be set
 * @param  d the day value to be set
 * @param  dw the day-week value to be set
 * @retval None
 */
void RTC_DateRegulate(RTC_HandleTypeDef* rtcHandle, uint8_t y, uint8_t m, uint8_t d, uint8_t dw)
{
	RTC_DateTypeDef sdatestructure;

	sdatestructure.Year = y;
	sdatestructure.Month = m;
	sdatestructure.Date = d;
	sdatestructure.WeekDay = dw;

	if (HAL_RTC_SetDate(rtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
	{
		/* Initialization Error */
		HAL_RTC_MspDeInit(rtcHandle);
		HAL_RTC_MspInit(rtcHandle);
	}
}

/**
 * @brief  Configures the current time
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(RTC_HandleTypeDef* rtcHandle, uint8_t hh, uint8_t mm, uint8_t ss, uint32_t Format)
{
	RTC_TimeTypeDef stimestructure;

	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.Hours = hh;
	stimestructure.Minutes = mm;
	stimestructure.Seconds = ss;
	stimestructure.SubSeconds = 0;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
/*	if (HAL_RTC_DST_ReadStoreOperation(rtcHandle))
		stimestructure.StoreOperation = RTC_STOREOPERATION_SET;
	else
	{
		stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_SUB1H;
		stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
	} */

	if (HAL_RTC_SetTime(rtcHandle, &stimestructure, Format) != HAL_OK)
	{
		/* Initialization Error */
		HAL_RTC_MspDeInit(rtcHandle);
		HAL_RTC_MspInit(rtcHandle);
	}
}

void enable_backup_sram(void)
{
	/*PWREN : Enable backup domain access; Enable the PWR clock */
	__HAL_RCC_PWR_CLK_ENABLE();
	/*DBP : Enable access to Backup domain */
	HAL_PWR_EnableBkUpAccess();
	/*BRE : Enable backup regulator */
//	HAL_PWREx_EnableBkUpReg();
}

void disable_backup_sram(void)
{
	/*PWREN : Disable backup domain access  */
	__HAL_RCC_PWR_CLK_DISABLE();
	/*DBP : Disable access to Backup domain */
	HAL_PWR_DisableBkUpAccess();
	/*BRE : Disable backup regulator */
//	HAL_PWREx_DisableBkUpReg();
}

/**
  * @brief  Writes a buffer in the SRam Backup data register.
  * @param  data: pointer to the data structure to be written in the SRam Backup data register.
  * @param  bytes: number of bytes to write in the SRam Backup data register.
  * @param  offset: is the displacement of the RTC Backup data register from where "data" will be written
  *			It can take values from 0 .. ((SRAM_BKUP_SIZE / 2) -1)
  * @retval 0 = Success
  */
int8_t writeBkpSram(uint8_t *data, uint16_t bytes, uint16_t offset)
{
	uint8_t* base_addr = (uint8_t *) BKPSRAM_BASE;
	uint16_t i;

	if( bytes + offset >= SRAM_BKUP_SIZE)
	{
		/* ERROR : the last byte is outside the backup SRAM region */
		return -1;
	}

	/* Enable clock to BKPSRAM */
	__HAL_RCC_BKPSRAM_CLK_ENABLE();
	/* Pointer write on specific location of backup SRAM */
	for( i = 0; i < bytes; i++ )
	{
		*(base_addr + offset + i) = *(data + i);
	}
	/* Disable clock to BKPSRAM */
	__HAL_RCC_BKPSRAM_CLK_DISABLE();

	return 0;
}

/**
  * @brief  Reads a buffer from the SRam Backup data register.
  * @param  data: pointer to the data structure to be read from the SRam Backup data register.
  * @param  bytes: number of bytes to read from the SRam Backup data register.
  * @param  offset: is the displacement of the SRam Backup data register from where values will be read
  *			It can take values from 0 .. ((SRAM_BKUP_SIZE / 2) -1)
  * @retval 0 = Success
  */
int8_t readBkpSram(uint8_t *data, uint16_t bytes, uint16_t offset)
{
	uint8_t* base_addr = (uint8_t *) BKPSRAM_BASE;
	uint16_t i;

	if( bytes + offset >= SRAM_BKUP_SIZE)
	{
		/* ERROR : the last byte is outside the backup SRAM region */
		return -1;
	}

	/* Enable clock to BKPSRAM */
	__HAL_RCC_BKPSRAM_CLK_ENABLE();
	/* Pointer write from specific location of backup SRAM */
	for( i = 0; i < bytes; i++ )
	{
		*(data + i) = *(base_addr + offset + i);
	}
	/* Disable clock to BKPSRAM */
	__HAL_RCC_BKPSRAM_CLK_DISABLE();

	return 0;
}

/**
  * @brief  Writes a buffer in the RTC Backup data register.
  * @param  data: pointer to the data structure to be written in the RTC Backup data register.
  * @param  bytes: number of bytes to write in the RTC Backup data register.
  * 		Since the RTC Backup data register is made up of 32bit registers, it must be a multiple of 4
  * @param  offset: is the displacement of the RTC Backup data register from where "data" will be written
  *			It can take values from 0 .. ((RTC_BKUP_SIZE / 2) -1)
  * @retval 0 = Success. Warning!!! Not Tested By Me!!!
  */
int8_t writeBkpRTC(uint8_t *data, uint16_t bytes, uint16_t offset)
{
	const uint16_t backup_size = 80;
	volatile uint32_t* base_addr = &(RTC->BKP0R);

	uint16_t i;

	if( bytes + offset >= backup_size)
	{
	/* ERROR : the last byte is outside the backup SRAM region */
		return -1;
	} else if( offset % 4 || bytes % 4 )
	{
	/* ERROR: data start or num bytes are not word aligned */
	return -2;
	} else
	{
		bytes >>= 2;      /* divide by 4 because writing words */
	}

	/* disable backup domain write protection */
	for( i = 0; i < bytes; i++ )
	{
		*(base_addr + offset + i) = *(data + i);
	}

	return 0;
}

/**
  * @brief  Reads a buffer from the RTC Backup data register.
  * @param  data: pointer to the data structure to be read from the RTC Backup data register.
  * @param  bytes: number of bytes to read from the RTC Backup data register.
  * 		Since the RTC Backup data register is made up of 16bit registers, it must be a multiple of 4
  * @param  offset: is the displacement of the RTC Backup data register from where values will be read
  *			It can take values from 0 .. ((RTC_BKUP_SIZE / 2) -1)
  * @retval 0 = Success. Warning!!! Not Tested By Me!!!
  */
int8_t readBkpRTC( uint8_t *data, uint16_t bytes, uint16_t offset)
{
	const uint16_t backup_size = 80;
	volatile uint32_t* base_addr = &(RTC->BKP0R);
	uint16_t i;

	if( bytes + offset >= backup_size )
	{
	/* ERROR : the last byte is outside the backup SRAM region */
		return -1;
	} else if( offset % 4 || bytes % 4 )
	{
		/* ERROR: data start or num bytes are not word aligned */
		return -2;
	} else
	{
		bytes >>= 2;      /* divide by 4 because writing words */
	}

	/* read should be 32 bit aligned */
	for( i = 0; i < bytes; i++ )
	{
		*(data + i) = *(base_addr + offset + i);
	}
	return 0;
}
/* USER CODE END 1 */
