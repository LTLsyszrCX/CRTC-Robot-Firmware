#include "PS2.h"

PS2Buttons *PS2_PS2;
TIM_HandleTypeDef *PS2_TIMER;

#define PS2_DELAY_TICKS 1u

#define PS2_TIMEOUT_TICKS 20000u

void delay_us(uint16_t us) {
	uint32_t start;
	uint32_t guard = 0u;
	us = (uint16_t)(us * PS2_DELAY_TICKS);
	if (us == 0u) {
		us = 1u;
	}
	__HAL_TIM_SET_COUNTER(PS2_TIMER, 0);
	start = __HAL_TIM_GET_COUNTER(PS2_TIMER);

	while ((uint32_t)(__HAL_TIM_GET_COUNTER(PS2_TIMER) - start) < us) {
		if (++guard >= PS2_TIMEOUT_TICKS) {
			break;
		}
	}
}

uint8_t PS2_SendByte(uint8_t cmd) {
	uint8_t data = 0x00;
	uint16_t buffer = 0x01;
	for (buffer = 0x01; buffer < 0x0100; buffer <<= 1) {
		if (buffer & cmd) {
			HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
		delay_us(15);
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
		delay_us(15);
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
		if (HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin)) {
			data |= buffer;
		}
	}
	delay_us(15);
	return data;
}

void PS2_Cmd(uint8_t *transmit, uint8_t *receive, uint8_t length) {
	uint32_t t0;
	uint32_t cost;
	__HAL_TIM_SET_COUNTER(PS2_TIMER, 0);
	HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_RESET);
	t0 = __HAL_TIM_GET_COUNTER(PS2_TIMER);
	delay_us(15);

	cost = __HAL_TIM_GET_COUNTER(PS2_TIMER) - t0;
	if (cost > 200u) {

		HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_SET);
		return;
	}
	for (uint8_t i = 0; i < length; i++) {
		receive[i] = PS2_SendByte(transmit[i]);
	}
	HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_SET);
	delay_us(15);
}

void PS2_Poll() {
	uint8_t transmit[5] = { 0x01, 0x42 };
	uint8_t receive[5] = { 0x00 };
	PS2_Cmd(transmit, receive, 5);
}

void PS2_EnterConfig() {
	uint8_t transmit[5] = { 0x01, 0x43, 0x00, 0x01 };
	uint8_t receive[5] = { 0x00 };
	PS2_Cmd(transmit, receive, 5);
}

void PS2_AnalogMode() {
	uint8_t transmit[9] = { 0x01, 0x44, 0x00, 0x01, 0x03 };
	uint8_t receive[9] = { 0x00 };
	PS2_Cmd(transmit, receive, 9);
}

void PS2_ExitConfig() {
	uint8_t transmit[9] = { 0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
	uint8_t receive[9] = { 0x00 };
	PS2_Cmd(transmit, receive, 9);
}

volatile uint8_t ps2_valid = 0;
static uint32_t  ps2_last_good_ms = 0;

#define PS2_HOLD_MS      200u
#define PS2_REINIT_MS    3000u

static uint8_t PS2_ProbeAnalog(void) {
    uint8_t tx[9] = { 0x01, 0x42 };
    uint8_t rx[9] = { 0x00 };

    PS2_Cmd(tx, rx, 9);
    return (rx[1] == 0x73) ? 1u : 0u;
}

static void PS2_SetNeutral(void) {
    if (PS2_PS2 == 0) {
        return;
    }
    PS2_PS2->SELECT = 0; PS2_PS2->L3 = 0; PS2_PS2->R3 = 0; PS2_PS2->START = 0;
    PS2_PS2->UP = 0; PS2_PS2->RIGHT = 0; PS2_PS2->DOWN = 0; PS2_PS2->LEFT = 0;
    PS2_PS2->L2 = 0; PS2_PS2->R2 = 0; PS2_PS2->L1 = 0; PS2_PS2->R1 = 0;
    PS2_PS2->TRIANGLE = 0; PS2_PS2->CIRCLE = 0; PS2_PS2->CROSS = 0; PS2_PS2->SQUARE = 0;
    PS2_PS2->RX = 128;
    PS2_PS2->RY = 128;
    PS2_PS2->LX = 128;
    PS2_PS2->LY = 128;
}

static void PS2_OnBadFrame(void) {
    uint32_t now = HAL_GetTick();

    if ((now - ps2_last_good_ms) > PS2_HOLD_MS) {
        ps2_valid = 0;
        PS2_SetNeutral();
    }

    if ((now - ps2_last_good_ms) > PS2_REINIT_MS) {
        ps2_last_good_ms = now;
        PS2_Init(PS2_TIMER, PS2_PS2);
    }
}

void PS2_Init(TIM_HandleTypeDef *timer, PS2Buttons *ps2) {
	uint8_t attempt;

	PS2_TIMER = timer, PS2_PS2 = ps2;

	ps2_valid = 0;
	ps2_last_good_ms = HAL_GetTick();
	PS2_SetNeutral();

	HAL_TIM_Base_Start(PS2_TIMER);

	delay_us(800);
	if (PS2_ProbeAnalog()) {
		return;
	}
	delay_us(800);
	if (PS2_ProbeAnalog()) {
		return;
	}

	for (attempt = 0; attempt < 3u; attempt++) {
		PS2_Poll();
		delay_us(800);
		PS2_EnterConfig();
		delay_us(800);
		PS2_AnalogMode();
		delay_us(800);
		PS2_Poll();
		delay_us(800);
		PS2_ExitConfig();
		delay_us(800);

		if (PS2_ProbeAnalog()) {
			break;
		}
		delay_us(800);
	}
}

void PS2_Update() {
	uint8_t transmit[9] = { 0x01, 0x42 };
	uint8_t receive[9] = { 0x00 };
	PS2_Cmd(transmit, receive, 9);

	if (receive[1] != 0x73) {
		PS2_OnBadFrame();
		return;
	}

	ps2_last_good_ms = HAL_GetTick();
	ps2_valid = 1;

	uint16_t buttonStatus = receive[3] | (receive[4] << 8);
	PS2_PS2->SELECT = !((buttonStatus >> 0) & 0x01);
	PS2_PS2->L3 = !((buttonStatus >> 1) & 0x01);
	PS2_PS2->R3 = !((buttonStatus >> 2) & 0x01);
	PS2_PS2->START = !((buttonStatus >> 3) & 0x01);
	PS2_PS2->UP = !((buttonStatus >> 4) & 0x01);
	PS2_PS2->RIGHT = !((buttonStatus >> 5) & 0x01);
	PS2_PS2->DOWN = !((buttonStatus >> 6) & 0x01);
	PS2_PS2->LEFT = !((buttonStatus >> 7) & 0x01);
	PS2_PS2->L2 = !((buttonStatus >> 8) & 0x01);
	PS2_PS2->R2 = !((buttonStatus >> 9) & 0x01);
	PS2_PS2->L1 = !((buttonStatus >> 10) & 0x01);
	PS2_PS2->R1 = !((buttonStatus >> 11) & 0x01);
	PS2_PS2->TRIANGLE = !((buttonStatus >> 12) & 0x01);
	PS2_PS2->CIRCLE = !((buttonStatus >> 13) & 0x01);
	PS2_PS2->CROSS = !((buttonStatus >> 14) & 0x01);
	PS2_PS2->SQUARE = !((buttonStatus >> 15) & 0x01);
	PS2_PS2->RX = receive[5];
	PS2_PS2->RY = receive[6];
	PS2_PS2->LX = receive[7];
	PS2_PS2->LY = receive[8];
}
