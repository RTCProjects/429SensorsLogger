#include "bsp_rtc.h"

RTC_HandleTypeDef RtcHandle;

static	RTC_TimeTypeDef	rtcTime;

void	BSP_RTC_Init()
{
	RtcHandle.Instance = RTC;
	RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
	RtcHandle.Init.AsynchPrediv = 0x7F;
	RtcHandle.Init.SynchPrediv = 0x00FF;
	RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

	  __HAL_RTC_RESET_HANDLE_STATE(&RtcHandle);

	if(HAL_RTC_Init(&RtcHandle) != HAL_OK){
	    _Error_Handler("bsp_rtc.h",20);
	}

	if(HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0) != 0x32F2){
		RTC_DateTypeDef  sdatestructure;
		RTC_TimeTypeDef  stimestructure;

		sdatestructure.Year = 0x00;
		  sdatestructure.Month = RTC_MONTH_JANUARY;
		  sdatestructure.Date = 0x00;
		  sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;

		  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
		  {
		    /* Initialization Error */
			_Error_Handler("bsp_rtc.h",35);
		  }

		  stimestructure.Hours = 0x00;
		    stimestructure.Minutes = 0x00;
		    stimestructure.Seconds = 0x00;
		    stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
		    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

		    if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
		    {
		      /* Initialization Error */
		    	_Error_Handler("bsp_rtc.h",48);
		    }

		    /*##-3- Writes a data in a RTC Backup data Register1 #######################*/
		    HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, 0x32F2);
	}

}

RTC_TimeTypeDef	*BSP_RTC_GetTime()
{
	HAL_RTC_GetTime(&RtcHandle, &rtcTime, RTC_FORMAT_BCD);

	return &rtcTime;
}

