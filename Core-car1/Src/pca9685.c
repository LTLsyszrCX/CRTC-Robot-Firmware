#include "pca9685.h"
#include "i2c.h"
#include "cmsis_os.h"

#define PCA9685_ADDR      0x80u
#define PCA9685_MODE1     0x00u
#define PCA9685_MODE2     0x01u
#define PCA9685_PRESCALE  0xFEu

#define PCA9685_MODE1_SLEEP   0x10u
#define PCA9685_MODE1_AI      0x20u
#define PCA9685_MODE2_OUTDRV  0x04u
#define PCA9685_PRESCALE_50HZ 0x79u

#define PCA9685_I2C_TIMEOUT    5u
#define PCA9685_RETRY          1u
#define PCA9685_RECOVER_AFTER  3u

volatile uint8_t  pca9685_ok = 0;
volatile uint8_t  pca9685_last_status = 0xFFu;
volatile uint16_t pca9685_errors = 0;
volatile uint16_t pca9685_recoveries = 0;

static uint16_t last_off[16];
static uint16_t fail_run = 0;
static uint8_t  in_recovery = 0;
static uint8_t  pca_ready = 0;

static void PCA9685_WriteBuf(uint8_t reg, const uint8_t *buf, uint16_t len);
static void PCA9685_Recover(void);
void PCA9685_Init(void);

static void i2c_delay_us(void) {
    for (volatile uint32_t i = 0; i < 60u; i++) {
        __NOP();
    }
}

static void PCA9685_Note(HAL_StatusTypeDef st) {
    pca9685_last_status = (uint8_t)st;

    if (st == HAL_OK) {
        fail_run = 0;
        pca9685_ok = 1;
        return;
    }

    if (pca9685_errors < 0xFFFFu) {
        pca9685_errors++;
    }
    if (fail_run < 0xFFFFu) {
        fail_run++;
    }
    pca9685_ok = 0;

    if (!in_recovery && pca_ready && fail_run >= PCA9685_RECOVER_AFTER) {
        fail_run = 0;
        PCA9685_Recover();
    }
}

static void PCA9685_WriteBuf(uint8_t reg, const uint8_t *buf, uint16_t len) {
    uint8_t  attempt;
    HAL_StatusTypeDef st = HAL_ERROR;

    for (attempt = 0; attempt <= PCA9685_RETRY; attempt++) {
        st = HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDR, reg, 1,
                               (uint8_t *)buf, len, PCA9685_I2C_TIMEOUT);
        if (st == HAL_OK) {
            break;
        }
    }
    PCA9685_Note(st);
}

static void PCA9685_WriteReg(uint8_t reg, uint8_t data) {
    PCA9685_WriteBuf(reg, &data, 1);
}

static void PCA9685_BusClear(void) {
    GPIO_InitTypeDef gi = {0};

    HAL_I2C_DeInit(&hi2c1);

    gi.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    gi.Mode  = GPIO_MODE_OUTPUT_OD;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gi);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    i2c_delay_us();

    for (uint8_t i = 0; i < 9u; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        i2c_delay_us();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        i2c_delay_us();
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    i2c_delay_us();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    i2c_delay_us();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    i2c_delay_us();

    HAL_I2C_Init(&hi2c1);
    fail_run = 0;
}

static void PCA9685_ConfigChip(void) {
    PCA9685_WriteReg(PCA9685_MODE1, PCA9685_MODE1_SLEEP);
    PCA9685_WriteReg(PCA9685_PRESCALE, PCA9685_PRESCALE_50HZ);
    PCA9685_WriteReg(PCA9685_MODE2, PCA9685_MODE2_OUTDRV);
    osDelay(1);
    PCA9685_WriteReg(PCA9685_MODE1, PCA9685_MODE1_AI);
}

#define PCA9685_RESTORE_CHANNELS   5u

static void PCA9685_RestorePositions(void) {
    for (uint8_t ch = 0; ch < PCA9685_RESTORE_CHANNELS; ch++) {
        PCA9685_SetPWM(ch, last_off[ch]);
    }
}

static void PCA9685_Recover(void) {
    in_recovery = 1;
    PCA9685_BusClear();
    PCA9685_ConfigChip();
    PCA9685_RestorePositions();
    in_recovery = 0;

    if (pca9685_recoveries < 0xFFFFu) {
        pca9685_recoveries++;
    }
}

void PCA9685_Init(void) {
    pca_ready = 0;
    pca9685_ok = 0;

    for (uint8_t ch = 0; ch < 16u; ch++) {
        last_off[ch] = 0u;
    }

    PCA9685_ConfigChip();

    for (uint8_t ch = 0; ch < 16u; ch++) {
        PCA9685_SetPWM(ch, 0u);
    }
    PCA9685_WriteReg(PCA9685_MODE1, PCA9685_MODE1_AI);

    pca_ready = 1;
    pca9685_ok = (pca9685_last_status == 0u) ? 1u : 0u;
}

void PCA9685_Rewake(void) {
    PCA9685_ConfigChip();
    PCA9685_RestorePositions();
}

void PCA9685_CheckAndRepair(void) {
    uint8_t mode1 = 0u;
    uint8_t mode2 = 0u;
    HAL_StatusTypeDef st1;
    HAL_StatusTypeDef st2;
    uint8_t healthy;

    st1 = HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDR, PCA9685_MODE1, 1,
                           &mode1, 1u, PCA9685_I2C_TIMEOUT);
    st2 = HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDR, PCA9685_MODE2, 1,
                           &mode2, 1u, PCA9685_I2C_TIMEOUT);

    PCA9685_Note(st1);
    PCA9685_Note(st2);

    if ((st1 != HAL_OK) || (st2 != HAL_OK)) {
        return;
    }

    healthy = (uint8_t)((((mode1 & (PCA9685_MODE1_SLEEP | PCA9685_MODE1_AI))
                          == PCA9685_MODE1_AI)
                         && ((mode2 & PCA9685_MODE2_OUTDRV) == PCA9685_MODE2_OUTDRV))
                        ? 1u : 0u);

    if (!healthy) {
        PCA9685_Rewake();
        if (pca9685_recoveries < 0xFFFFu) {
            pca9685_recoveries++;
        }
    }
}

void PCA9685_SetPWM(uint8_t ch, uint16_t off) {
    uint8_t base_reg;
    uint8_t data[4];

    if (ch > 15u) {
        return;
    }
    if (off > 4095u) {
        off = 4095u;
    }

    last_off[ch] = off;

    base_reg = (uint8_t)(0x06u + 4u * ch);
    data[0] = 0x00u;
    data[1] = 0x00u;
    data[2] = (uint8_t)(off & 0xFFu);
    data[3] = (uint8_t)((off >> 8) & 0x0Fu);

    PCA9685_WriteBuf(base_reg, data, 4u);
}
