#include "SD.h"

static void UART_Print(char* str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *) str, strlen(str), 100);
}

void SD_Write(void* data, uint8_t len, char* file){
	FIL Fil;
	FRESULT FR_Status;
	UINT WWC;

	//------------------[ Mount The SD Card ]--------------------
	FR_Status = f_mount(&USERFatFS, "", 1);
	if (FR_Status != FR_OK)
	{
	  UART_Print("MOUNT ERROR\n");
	}
	else {
		UART_Print("MOUNT SUCCSES\n");
	}
	FR_Status = f_open(&Fil, file , FA_WRITE | FA_READ | FA_OPEN_EXISTING);
	uint32_t size = f_size(&Fil);
	f_lseek(&Fil, size);
	if(FR_Status != FR_OK)
	{
	  UART_Print("OPEN ERROR\n");
	}

	FR_Status = f_write(&Fil, data, len, &WWC);
	if (FR_Status != FR_OK) {
		UART_Print("WRITE ERROR\n");
	}
	else {
		UART_Print("WRITE SUCCES\n");
	}
	f_close(&Fil);

	FR_Status = f_mount(NULL, "", 0);
	if (FR_Status != FR_OK)
	{
		UART_Print("UNMOUNT ERROR\n");
	} else{
		UART_Print("UNMOUNT SUCCSESFULLY\n");
	}
}
