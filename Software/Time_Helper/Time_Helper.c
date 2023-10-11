#include <stm32f10x.h>

char* time_to_timestamp(void)
{
	unsigned int timestamp;
	struct tm stm;
	char *timestamp_buf;
	char *buf;
	timestamp_buf = (char *)mem_malloc(10);
	buf = (char *)mem_malloc(100);
    
	RTC_DateTypeDef RTC_DateStruct; 
	RTC_TimeTypeDef	RTC_TimeStruct;
	RTC_GetDate(RTC_Format_BIN,&RTC_DateStruct);
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);
	
	stm.tm_year = RTC_DateStruct.RTC_Year + 100;	//RTC_Year rang 0-99,but tm_year since 1900
	stm.tm_mon	= RTC_DateStruct.RTC_Month-1;		//RTC_Month rang 1-12,but tm_mon rang 0-11
	stm.tm_mday	= RTC_DateStruct.RTC_Date;			//RTC_Date rang 1-31 and tm_mday rang 1-31
	stm.tm_hour	= RTC_TimeStruct.RTC_Hours-8;			//RTC_Hours rang 0-23 and tm_hour rang 0-23
	stm.tm_min	= RTC_TimeStruct.RTC_Minutes;   //RTC_Minutes rang 0-59 and tm_min rang 0-59
	stm.tm_sec	= RTC_TimeStruct.RTC_Seconds;		
	sprintf(buf,"\r\nrtc %d-%d-%d  %d.%d.%d\r\n",\
	RTC_DateStruct.RTC_Year,RTC_DateStruct.RTC_Month,RTC_DateStruct.RTC_Date,\
	RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds);
	USART_COM3_Send_data(buf,strlen(buf));
	mem_free(buf);
	sprintf(timestamp_buf,"%u",mktime(&stm));
	USART_COM3_Send_data(timestamp_buf,strlen(timestamp_buf));
	return timestamp_buf;
}

void timestamp_to_time(unsigned int timestamp)
{
	struct tm *stm= NULL;
	char *buf;
	buf = (char *)mem_malloc(100);
	
	RTC_DateTypeDef RTC_DateStruct;
	RTC_TimeTypeDef	RTC_TimeStruct;
	
	stm = localtime(×tamp);
	RTC_DateStruct.RTC_Year = stm->tm_year - 100;
	RTC_DateStruct.RTC_Month = stm->tm_mon + 1;
	RTC_DateStruct.RTC_Date = stm->tm_mday;
	RTC_TimeStruct.RTC_Hours = stm->tm_hour + 8;
	RTC_TimeStruct.RTC_Minutes = stm->tm_min;
	RTC_TimeStruct.RTC_Seconds = stm->tm_sec;
	
	sprintf(buf,"\r\nstm %d-%d-%d  %d.%d.%d\r\n",\
	stm->tm_year,stm->tm_mon,stm->tm_mday,\
	stm->tm_hour,stm->tm_min,stm->tm_sec);
	USART_COM3_Send_data(buf,strlen(buf));
	mem_free(buf);
	
	RTC_SetDate(RTC_Format_BIN,&RTC_DateStruct);
	RTC_SetTime(RTC_Format_BIN,&RTC_TimeStruct);
}