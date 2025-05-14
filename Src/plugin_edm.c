// SPDX-License-Identifier: AGPL-3.0-or-later
#include "driver.h"  // hal.*, driver macros

#if EDM_ENABLE

#include "grbl/core_handlers.h"
#include "grbl/grbl.h"
#include "i2c.h"
#include "platform.h"

#include <stdio.h>

/* ------------------------------------------------------------------ */
/* ===== USER ADJUSTABLES =========================================== */
#define EDM_I2C_GPIO GPIOB
#define EDM_I2C_SCL_PIN 8
#define EDM_I2C_SDA_PIN 9
#define EDM_I2C I2C4                              // PB8/PB9 is I2C4
#define EDM_I2C_AF GPIO_AF4_I2C4                  // TODO: auto-gen
#define EDM_I2C_CLKENA __HAL_RCC_I2C4_CLK_ENABLE  // TODO: auto-gen
#define EDM_I2C_IRQEVT I2C4_EV_IRQn               // TODO: auto-gen
#define EDM_I2C_IRQERR I2C4_ER_IRQn               // TODO: auto-gen

#define EDM_ADDR 0x3b       // <-- change to suit
#define EDM_REG_FIRST 0x00  // first register to read
#define EDM_REG_COUNT 3     // how many bytes

#define EDM_MCODE_READ 550

/* ------------------------------------------------------------------ */

// /* one “latest” snapshot – extend later if you want to buffer */
// static uint8_t edm_snapshot[EDM_REG_COUNT] = {0};

// /* ------------------------------------------------------------------ */
// /* 1 kHz polling ISR – keep it *short* (called from a HAL timer IRQ)   */
// static void edm_poll_isr(void* data) {
//   uint8_t buf[EDM_REG_COUNT];
//   if (i2c_read(EDM_I2C, EDM_ADDR, EDM_REG_FIRST, buf, EDM_REG_COUNT))
//     memcpy(edm_snapshot, buf, EDM_REG_COUNT);  // update the live copy
// }

// /* ------------------------------------------------------------------ */
// /* ----------  USER-MCODE HANDLERS  --------------------------------- */

static user_mcode_ptrs_t other_mcode_ptrs;

static user_mcode_type_t mcode_check(user_mcode_t m) {
  if (m != EDM_MCODE_READ) {
    return other_mcode_ptrs.check ? other_mcode_ptrs.check(m)
                                  : UserMCode_Unsupported;
  }

  return UserMCode_Normal;
}

static status_code_t mcode_validate(parser_block_t* block) {
  if (block->user_mcode != EDM_MCODE_READ) {
    return other_mcode_ptrs.validate ? other_mcode_ptrs.validate(block)
                                     : Status_Unhandled;
  }

  return Status_OK;
}

static void mcode_execute(uint_fast16_t state, parser_block_t* block) {
    if (block->user_mcode != EDM_MCODE_READ) {
        if (other_mcode_ptrs.execute) {
            other_mcode_ptrs.execute(state, block);
        }
        return;
    }
    
    char reply[40];
    /* `[EDM:xx,yy,zz]` (expand as required) */
    snprintf(reply, sizeof(reply), "[EDM:Hello]" ASCII_EOL);
    hal.stream.write(reply);
}

/* ------------------------------------------------------------------ */
/* ---------------- PLUGIN INITIALISATION --------------------------- */

static I2C_HandleTypeDef edm_i2c_port = {
    .Instance = EDM_I2C,
    .Init.Timing = 0x00B03FDB,  // 400 kHz
    .Init.OwnAddress1 = 0,
    .Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    .Init.DualAddressMode = I2C_DUALADDRESS_DISABLE,
    .Init.OwnAddress2 = 0,
    .Init.GeneralCallMode = I2C_GENERALCALL_DISABLE,
    .Init.NoStretchMode = I2C_NOSTRETCH_DISABLE};

void I2C_IRQEVT_Handler(void) {
  HAL_I2C_EV_IRQHandler(&edm_i2c_port);
}

void I2C_IRQERR_Handler(void) {
  HAL_I2C_ER_IRQHandler(&edm_i2c_port);
}

void edm_init() {
  // init i2c
  GPIO_InitTypeDef init_struct = {
      .Pin = (1 << EDM_I2C_SCL_PIN) | (1 << EDM_I2C_SDA_PIN),
      .Mode = GPIO_MODE_AF_OD,
      .Pull = GPIO_PULLUP,
      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
      .Alternate = EDM_I2C_AF};
  HAL_GPIO_Init(EDM_I2C_GPIO, &init_struct);

  EDM_I2C_CLKENA();

  HAL_I2C_Init(&edm_i2c_port);
  HAL_I2CEx_ConfigAnalogFilter(&edm_i2c_port, I2C_ANALOGFILTER_ENABLE);

  HAL_NVIC_EnableIRQ(EDM_I2C_IRQEVT);
  HAL_NVIC_EnableIRQ(EDM_I2C_IRQERR);

  static const periph_pin_t scl = {.function = Output_SCK,
                                   .group = PinGroup_I2C,
                                   .port = EDM_I2C_GPIO,
                                   .pin = EDM_I2C_SCL_PIN,
                                   .mode = {.mask = PINMODE_OD}};

  static const periph_pin_t sda = {.function = Bidirectional_SDA,
                                   .group = PinGroup_I2C,
                                   .port = EDM_I2C_GPIO,
                                   .pin = EDM_I2C_SDA_PIN,
                                   .mode = {.mask = PINMODE_OD}};

  hal.periph_port.register_pin(&scl);
  hal.periph_port.register_pin(&sda);

  /* ----------- 2.  1 kHz timer --------------------------------- */
  // timer_handle_t t = hal.timers.alloc(1000, true, edm_poll_isr, NULL);
  // hal.timers.start(t); /* fires every 1 ms */

  // Register M-code handler by appending to the call chain.
  memcpy(&other_mcode_ptrs, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
  grbl.user_mcode.check = mcode_check;
  grbl.user_mcode.validate = mcode_validate;
  grbl.user_mcode.execute = mcode_execute;

  /* optional hello */
  report_plugin("EDM", "0.0");
}

#endif /** EDM_ENABLE */
