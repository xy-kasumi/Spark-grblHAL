// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * Driver for Spark EDM.
 *
 * ## Supported M-codes
 *
 * M503 P[pulse_time_us] Q[pulse_current_a] R[max_duty]
 * Energize, tool negative voltage
 *
 * M504 P[pulse_time_us] Q[pulse_current_a] R[max_duty]
 * Energize, tool positive voltage
 *
 * M505
 * De-energize
 *
 * M550 S[output_log<optional>]]
 * Print EDM plugin status.
 * If S is omitted, prints general status.abort
 * If S is specified, print log.
 * For log printing to work, log must be disabled state (default or M551 S0).
 *
 * M551 S[log_enable<required>]
 * Control log status.
 * - log_enable: required. Must be 0 or 1.
 * Whenever M551 S1 is called, all previous log entries are cleared.
 *
 * ## Supported G-codes
 * G1: Enabled feed rate control & retract.
 * G38.2, G38.3: Probe using current sensing. De-energize (same as M505) on
 * contact or not-found completion. Need M503 or M504 before G38 to activate
 * current for probing.
 */
#if EDM_ENABLE

#include "driver.h"
#include "grbl/core_handlers.h"
#include "grbl/grbl.h"
#include "i2c.h"
#include "platform.h"

#include <math.h>
#include <stdio.h>

#define EDM_MCODE_START_TNEG 503
#define EDM_MCODE_START_TPOS 504
#define EDM_MCODE_STOP 505
#define EDM_MCODE_READ 550
#define EDM_MCODE_LOG 551

// You can change this if you somehow want to use Aux8 for different purposes.
// If you change this, you also need to updard board configuration header.
#define PIN_FUNCTION_PULSER_GATE Output_Aux8

// See https://github.com/xy-kasumi/Spark/blob/main/docs/user-PULSER.md for
// register map
#define PULSER_ADDR 0x3b

static const uint8_t REG_POLARITY = 0x01;
static const uint8_t REG_PULSE_CURRENT = 0x02;
static const uint8_t REG_TEMPERATURE = 0x03;
static const uint8_t REG_PULSE_DUR = 0x04;
static const uint8_t REG_MAX_DUTY = 0x05;
static const uint8_t REG_CKP_N_PULSE = 0x10;

// Simple status flag for debugging initialization error.
// This will be read by M-code.
// 0: OK
// 255: initial value
// You can use whatever other numbers to indicate failure modes.
static volatile uint8_t edm_init_status = 255;

static volatile uint32_t edm_poll_cnt = 0;
static volatile bool edm_has_current = false;
static volatile uint64_t last_poll_tick_us;  // hal.get_micros() time

static volatile bool edm_removal_active = false;

#define EDM_LOG_SIZE 10000  // 10 sec

const uint8_t ST_MOTION = 0x01;  // corresponds to execute_sys_motion

typedef struct {
  uint8_t status_flags;
  uint8_t r_open;
  uint8_t r_short;
  uint8_t r_pulse;
  uint8_t n_pulse;
} log_entry_t;

typedef struct {
  log_entry_t entries[EDM_LOG_SIZE];
  int ix_write;
  int num_valid;
  bool active;
} edm_log_t;

static volatile edm_log_t edm_log;

static void init_log() {
  edm_log.ix_write = 0;
  edm_log.num_valid = 0;
  edm_log.active = false;
}

static void add_log(log_entry_t entry) {
  edm_log.entries[edm_log.ix_write] = entry;
  edm_log.ix_write = (edm_log.ix_write + 1) % EDM_LOG_SIZE;
  if (edm_log.num_valid < EDM_LOG_SIZE) {
    edm_log.num_valid++;
  }
}

////////////////////////////////////////////////////////////////////////////////
// M-code handlers

static on_probe_completed_ptr other_probe_completed;
static user_mcode_ptrs_t other_mcode_ptrs;

static void exec_mcode_read(bool print_log) {
  bool succ;

  uint8_t buf[1];
  i2c_transfer_t tx;
  tx.address = PULSER_ADDR;
  tx.word_addr = REG_TEMPERATURE;
  tx.word_addr_bytes = 1;
  tx.count = 1;
  tx.data = buf;
  tx.no_block = false;
  if (i2c_transfer(&tx, true)) {
    succ = true;
  } else {
    succ = false;
  }
  uint8_t temp = buf[0];

  char resp[100];
  size_t ofs = 0;
  ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "[EDM|stat=%d,",
                  edm_init_status);
  if (succ) {
    ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "i2c=ok,temp=%d", temp);
  } else {
    ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "i2c=fail");
  }
  ofs += snprintf(resp + ofs, sizeof(resp) - ofs, ",polls=%ld,log=%d",
                  edm_poll_cnt, edm_log.num_valid);

  ofs += snprintf(resp + ofs, sizeof(resp) - ofs, ",F(step)=%ldHz",
                  hal.f_step_timer);

  ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "]" ASCII_EOL);
  hal.stream.write(resp);

  if (print_log && !edm_log.active) {
    const int entry_per_line = 20;
    int ix_read =
        (edm_log.ix_write + EDM_LOG_SIZE - edm_log.num_valid) % EDM_LOG_SIZE;
    int num_lines = (edm_log.num_valid + entry_per_line - 1) / entry_per_line;
    for (int i = 0; i < num_lines; i++) {
      ofs = 0;
      ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "[EDML|");

      int n_pulse = 0;
      bool has_motion = false;
      for (int j = 0; j < entry_per_line; j++) {
        int ix_log = i * entry_per_line + j;
        if (ix_log >= edm_log.num_valid) {
          // no more log
          ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "00,");
        } else {
          // has entry
          log_entry_t entry = edm_log.entries[ix_read];
          if (entry.status_flags > 0) {
            has_motion = true;
          }
          n_pulse += entry.n_pulse;

          int v_pulse = (entry.r_pulse * 10) / 255;
          int v_short = (entry.r_short * 10) / 255;
          ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "%d%d,", v_pulse,
                          v_short);
        }
        ix_read = (ix_read + 1) % EDM_LOG_SIZE;
      }
      ofs += snprintf(resp + ofs, sizeof(resp) - ofs, has_motion ? "M," : "-,");
      ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "%d]" ASCII_EOL, n_pulse);
      hal.stream.write(resp);
    }
  }
}

static void exec_mode_log(bool enable) {
  if (enable) {
    init_log();
  }
  edm_log.active = enable;
}

// blocking I2C register write.
// returns true if write was successful
static bool write_reg(uint8_t reg_addr, uint8_t val) {
  i2c_transfer_t tx;
  tx.address = PULSER_ADDR;
  tx.word_addr = reg_addr;
  tx.word_addr_bytes = 1;
  tx.count = 1;
  tx.data = &val;
  tx.no_block = false;
  return i2c_transfer(&tx, false);
}

inline static void init_gate() {
  GPIO_InitTypeDef init = {
      .Pin = 1 << PULSER_GATE_PIN,
      .Speed = GPIO_SPEED_FREQ_MEDIUM,
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
  };
  HAL_GPIO_Init(PULSER_GATE_PORT, &init);
}

inline static void set_gate(bool on) {
  DIGITAL_OUT(PULSER_GATE_PORT, 1 << PULSER_GATE_PIN, on);
}

// must not be called when edm_init_status != 0
static void exec_mcode_start(bool tool_neg,
                             int pulse_dur_10us,
                             int pulse_current_100ma,
                             int pulse_duty_pct) {
  bool all_ok = true;
  all_ok &= write_reg(REG_PULSE_CURRENT, pulse_current_100ma);
  all_ok &= write_reg(REG_PULSE_DUR, pulse_dur_10us);
  all_ok &= write_reg(REG_MAX_DUTY, pulse_duty_pct);
  all_ok &= write_reg(REG_POLARITY, tool_neg ? 2 : 1);  // 2: T- W+, 1: T+ W-
  if (!all_ok) {
    system_raise_alarm(Alarm_SelftestFailed);
    return;
  }
  set_gate(true);
}

// must not be called when edm_init_status != 0
static void exec_mcode_stop() {
  set_gate(false);

  bool ok = write_reg(REG_POLARITY, 0);  // OFF
  if (!ok) {
    system_raise_alarm(Alarm_SelftestFailed);
    return;
  }
}

static inline bool is_edm_mcode(user_mcode_t m) {
  return (m == EDM_MCODE_READ || m == EDM_MCODE_START_TNEG ||
          m == EDM_MCODE_START_TPOS || m == EDM_MCODE_STOP ||
          m == EDM_MCODE_LOG);
}

static user_mcode_type_t mcode_check(user_mcode_t m) {
  if (!is_edm_mcode(m)) {
    return other_mcode_ptrs.check ? other_mcode_ptrs.check(m)
                                  : UserMCode_Unsupported;
  }

  return UserMCode_Normal;
}

static status_code_t mcode_validate(parser_block_t* block) {
  user_mcode_t code = block->user_mcode;

  if (!is_edm_mcode(code)) {
    return other_mcode_ptrs.validate ? other_mcode_ptrs.validate(block)
                                     : Status_Unhandled;
  }

  switch ((int)code) {
    case EDM_MCODE_READ:
      if (block->words.s) {
        block->words.s = 0;
      }
      block->user_mcode_sync = true;
      return Status_OK;
    case EDM_MCODE_LOG:
      if (!block->words.s) {
        return Status_GcodeValueWordMissing;
      }
      {
        float v = block->values.s;
        if (isnan(v) || (v != 0 && v != 1)) {
          return Status_GcodeValueOutOfRange;
        }
        block->words.s = 0;
      }
      block->user_mcode_sync = true;
      return Status_OK;
    case EDM_MCODE_STOP:
      if (block->words.mask != 0) {
        return Status_GcodeUnusedWords;
      }
      if (edm_init_status != 0) {
        return Status_SelfTestFailed;
      }
      block->user_mcode_sync = true;
      return Status_OK;
    default: {
      // P (pulse duration): 100us~1000us is allowed
      if (block->words.p) {
        float v = block->values.p;
        if (isnan(v) || v < 100 || v > 1000) {
          return Status_GcodeValueOutOfRange;
        }
        block->words.p = 0;
      }
      // Q (pulse current): 0(min)~20(A) is allowed
      if (block->words.q) {
        float v = block->values.q;
        if (isnan(v) || v < 0 || v > 20) {
          return Status_GcodeValueOutOfRange;
        }
        block->words.q = 0;
      }
      // R (duty factor): 1~95 is allowed
      if (block->words.r) {
        float v = block->values.r;
        if (isnan(v) || v < 1 || v > 95) {
          return Status_GcodeValueOutOfRange;
        }
        block->words.r = 0;
      }
      if (edm_init_status != 0) {
        return Status_SelfTestFailed;
      }
      block->user_mcode_sync = true;
      return Status_OK;
    }
  }
}

static void mcode_execute(uint_fast16_t state, parser_block_t* block) {
  user_mcode_t code = block->user_mcode;
  if (!is_edm_mcode(code)) {
    if (other_mcode_ptrs.execute) {
      other_mcode_ptrs.execute(state, block);
    }
    return;
  }

  if (code == EDM_MCODE_READ) {
    exec_mcode_read(block->words.s);
  } else if (code == EDM_MCODE_LOG) {
    bool enable = block->values.s > 0;
    exec_mode_log(enable);
  } else if (code == EDM_MCODE_START_TNEG || code == EDM_MCODE_START_TPOS) {
    bool is_tneg = (code == EDM_MCODE_START_TNEG);

    int pulse_dur_10us = 50;  // 500us default
    if (block->words.p) {
      pulse_dur_10us = block->values.p * 0.1f;
    }
    int pulse_current_100ma = 10;  // 1A default
    if (block->words.q) {
      pulse_current_100ma = block->values.q * 10;
      if (pulse_current_100ma == 0) {
        pulse_current_100ma = 1;  // 100mA (minimum)
      }
    }
    int pulse_duty_pct = 25;  // 25% default
    if (block->words.r) {
      pulse_duty_pct = block->values.r;
    }

    exec_mcode_start(is_tneg, pulse_dur_10us, pulse_current_100ma,
                     pulse_duty_pct);
  } else if (code == EDM_MCODE_STOP) {
    exec_mcode_stop();
  }
}

static void edm_probe_completed() {
  // stop discharge to minimize work damage.
  set_gate(false);

  bool ok = write_reg(REG_POLARITY, 0);  // OFF
  if (!ok) {
    system_raise_alarm(Alarm_SelftestFailed);
    return;
  }

  //
  if (other_probe_completed) {
    other_probe_completed();
  }
}

////////////////////////////////////////////////////////////////////////////////
// EDM Probe

void edm_probe_configure(bool is_probe_away, bool probing) {
  // Maybe nothing is needed here?
  // \param is_probe_away true if probing away from the workpiece, false
  // otherwise. When probing away the signal must be inverted in the
  // probe_get_state_ptr() implementation.
  // \param probing true if probe cycle is active, false otherwise.
}

void edm_probe_connected_toggle() {
  // do nothing; no notion of "toggle"
}

// NOTE: Called from stepping interrupt, must run extremely quickly.
probe_state_t edm_probe_get_state() {
  probe_state_t state = {0};
  state.triggered = edm_has_current;
  state.connected = true;  // always connected
  return state;
}

////////////////////////////////////////////////////////////////////////////////
// Timer

on_execute_realtime_ptr other_realtime;

static void edm_realtime(sys_state_t s) {
  if (other_realtime) {
    other_realtime(s);
  }

  // Limit to 1ms polling rate.
  uint64_t t_curr = hal.get_micros();
  if (t_curr - last_poll_tick_us < 1000) {
    return;
  }
  last_poll_tick_us = t_curr;

  // Do I2C poll
  uint8_t buf[6];
  i2c_transfer_t tx;
  tx.address = PULSER_ADDR;
  tx.word_addr = REG_CKP_N_PULSE;
  tx.word_addr_bytes = 1;
  tx.count = 6;
  tx.data = buf;
  tx.no_block = false;
  if (!i2c_transfer(&tx, true)) {
    return;
  }

  uint8_t n_pulse = buf[0];
  uint8_t r_pulse = buf[3];
  uint8_t r_short = buf[4];
  uint8_t r_open = buf[5];
  edm_has_current = (r_pulse > 0 || r_short > 0);

  if (r_short > 127) {
    // retract request
    hal.edm_state.discharge_short = true;
  }

  if (edm_log.active) {
    log_entry_t entry = {
        .status_flags = sys.step_control.execute_sys_motion ? ST_MOTION : 0,
        .r_open = r_open,
        .r_short = r_short,
        .r_pulse = r_pulse,
        .n_pulse = n_pulse,
    };
    add_log(entry);
  }
  edm_poll_cnt++;
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
  // Register report printer.
  other_reports = grbl.on_report_options;
  grbl.on_report_options = edm_report_options;

  // Init logging. This must come before edm_realtime starting.
  init_log();

  // Register M-code handler by appending to the call chain.
  memcpy(&other_mcode_ptrs, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
  grbl.user_mcode.check = mcode_check;
  grbl.user_mcode.validate = mcode_validate;
  grbl.user_mcode.execute = mcode_execute;

  i2c_start();

  init_gate();
  set_gate(false);  // ensure it's off

  // Register EDM virtual probe to HAL.
  hal.probe.configure = edm_probe_configure;
  hal.probe.connected_toggle = edm_probe_connected_toggle;
  hal.probe.get_state = edm_probe_get_state;

  // Register probe completed handler.
  other_probe_completed = grbl.on_probe_completed;
  grbl.on_probe_completed = edm_probe_completed;

  // Register PULSER polling as rate-limited "realtime" process.
  // This is better than hal.timers based approach.
  // Doing I2C in timer will cause TMC2209 init to fail, as they're bit-banging
  // serial comm is very sensitive to timing.
  other_realtime = grbl.on_execute_realtime;
  grbl.on_execute_realtime = edm_realtime;

  // Mark as OK.
  edm_init_status = 0;
}

#endif /** EDM_ENABLE */
