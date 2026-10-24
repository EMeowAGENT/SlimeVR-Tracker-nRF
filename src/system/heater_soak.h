#ifndef SYSTEM_HEATER_SOAK_H
#define SYSTEM_HEATER_SOAK_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Raw heater soak: drive the IMU heater for a bounded time without touching
 * T-Cal at all. Intended for stress-relief cycling and for proving that the
 * heater path works when a calibration session cannot complete.
 *
 * Closed-loop bang-bang around target_c (default 43 C) at up to duty_pptt.
 * Safety: aborts on stale temperature, on loss of external power, and at a
 * fixed ceiling below the hardware shutdown threshold.
 */

/* Closed loop: hold target_c, drive capped by CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT. */
int heater_soak_start(uint16_t minutes, float target_c);
/* Open loop: fixed duty_pptt, no temperature target (ceiling still enforced). */
int heater_soak_start_raw(uint16_t minutes, uint16_t duty_pptt);
void heater_soak_stop(void);
bool heater_soak_active(void);
void heater_soak_report(void);

#endif
