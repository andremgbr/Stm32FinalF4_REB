#include <unistd.h>
#include "ecu.h"
#include "main.h"
#include "mcal.h"
#include <string.h>

char buffer[200] = { 0 };

//void REPORT_ERROR(char * msg){
//	sprintf(buffer, "%c\r\n" , msg);
//	HAL_UART_Transmit(huart2, msg, len(msg), HAL_MAX_DELAY);
//}

/**
 *  @brief Get the status of desired PIN number.
 *
 *  @param status Pointer to store the read status of the PIN (0 or 1).
 *  @param pin Pointer of PIN number to be read.
 *  @return SUCCESS(0), FAIL(1)
 *  @requir{SwHLR_F_13}
 */
uint8_t read_pin_status(uint8_t *status, uint8_t pin) {
	uint8_t return_status = ECU_FAIL;
	*status =
			HAL_GPIO_ReadPin(GPIOC, pin) == GPIO_PIN_SET ? 0x01U : 0x00U;
	return_status = ECU_SUCCESS;
	return return_status;
}

/**
 *  @brief Set the status of desired PIN number.
 *
 *  @param status Pointer to store the status of the PIN (0 or 1).
 *  @param pin Pointer of PIN number to be read.
 *  @return SUCCESS(0), FAIL(1)
 *  @requir{SwHLR_F_13}
 */
uint8_t set_pin_status(uint8_t p_status, uint8_t p_pin) {
	uint8_t return_status = ECU_FAIL;
	HAL_GPIO_WritePin(GPIOC, p_pin, p_status == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	return ECU_SUCCESS;
}

/**
 *  @brief Sleep thread POSIX.
 *
 *  @param seconds How many seconds to sleep.
 */
void go_sleep(uint8_t seconds) {
	HAL_Delay(seconds * 1000);
}

void show_error(char *msg) {
	char buffer[300];
	sprintf(buffer, "%s \r\n", msg);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
}

void get_time(timespec *time) {
	uint32_t tick_ms = HAL_GetTick();
	time->tv_sec = tick_ms / 1000;
	time->tv_nsec = (tick_ms % 1000) * 1000000;
}

void show_log(char *msg) {
	char buffer[300];
	sprintf(buffer, "%s \r\n", msg);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
}

uint8_t can_send_vcan0(can_frame *frame) {

	uint32_t mailbox;

	CAN_TxHeaderTypeDef TxHeader;
	// Configure message
	TxHeader.StdId = frame->can_id; // Standard ID (11-bit)
	TxHeader.ExtId = 0x00; // Not used in standard ID mode
	TxHeader.IDE = CAN_ID_STD; // Standard identifier
	TxHeader.RTR = CAN_RTR_DATA; // Data frame (not remote)
	TxHeader.DLC = 8; // Data length (0-8 bytes)

	uint8_t data[8] = { 0 };
	for (int i = 0; i < 8; i++) {
		data[i] = frame->data[i];
	}

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) != HAL_OK) {
		char buffer[300];
		sprintf(buffer, "Error can_send_vcan0 \r\n");
		HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
	}

	return 0;
}
