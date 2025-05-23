// SPDX-License-Identifier: AGPL-3.0-or-later
#include "driver.h"  // hal.*, driver macros

#if EDM_ENABLE

#include "grbl/core_handlers.h"
#include "grbl/grbl.h"
#include "i2c.h"
#include "platform.h"

#include <stdio.h>


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

  bool succ;

  uint8_t buf[1];
  i2c_transfer_t tx;
  tx.address = EDM_ADDR;
  tx.word_addr = 0x03;
  tx.word_addr_bytes = 1;
  tx.count = 1;
  tx.data = buf;
  tx.no_block = false;
  if (i2c_transfer(&tx, true)) {
    succ = true;
  } else {
    succ = false;
  }

  char reply[40];
  if (succ) {
    snprintf(reply, sizeof(reply), "[EDM:I2C-OK: temp=%d]" ASCII_EOL, buf[0]);
  } else {
    snprintf(reply, sizeof(reply), "[EDM:FAIL]" ASCII_EOL);
  }
  hal.stream.write(reply);
}

////////////////////////////////////////////////////////////////////////////////
// Plugin Reporting

static on_report_options_ptr other_reports;

static void edm_report_options(bool newopt) {
  if (other_reports) {
    other_reports(newopt);
  }

  if (!newopt) {
    report_plugin("EDM", "0.0");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Plugin Init

void edm_init() {
  i2c_start();

  /* ----------- 2.  1 kHz timer --------------------------------- */
  // timer_handle_t t = hal.timers.alloc(1000, true, edm_poll_isr, NULL);
  // hal.timers.start(t); /* fires every 1 ms */

  // Register M-code handler by appending to the call chain.
  memcpy(&other_mcode_ptrs, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
  grbl.user_mcode.check = mcode_check;
  grbl.user_mcode.validate = mcode_validate;
  grbl.user_mcode.execute = mcode_execute;

  // Register report printer.
  other_reports = grbl.on_report_options;
  grbl.on_report_options = edm_report_options;
}

#endif /** EDM_ENABLE */
