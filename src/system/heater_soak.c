/*
 * Raw heater soak (no calibration).
 *
 * The heated T-Cal session owns the heater only after its admission and
 * sampling gates succeed, which makes it useless as a stress-relief tool when
 * a unit cannot complete that flow. This module drives the heater directly:
 * arm a lease, hold a target with bang-bang control, and never touch the
 * temperature-calibration table.
 */

#include "heater_soak.h"

#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_SENSOR_TCAL_HEATED)

#include "system/heater.h"
#include "system/power.h"
#include "system/test_mode.h"
#include "sensor/sensor.h"
#include "sensor/calibration/tcal_heated.h"

LOG_MODULE_REGISTER(heater_soak, LOG_LEVEL_INF);

#define SOAK_TICK_MS 1000
/* Software ceiling: below CONFIG_SENSOR_TCAL_HEATED_MAX_TEMP_C (50) so the
 * soak cannot sit on the hardware shutdown threshold. */
#define SOAK_CEILING_C 48.0f
#define SOAK_HYSTERESIS_C 0.3f
#define SOAK_LOGGING_MS 10000

static struct {
	bool active;
	bool closed_loop;
	uint32_t generation;
	uint32_t sequence;
	uint16_t max_duty;
	uint16_t duty;
	float target;
	int64_t started_ms;
	int64_t deadline_ms;
	int64_t next_log_ms;
	bool test_mode_was_on;
} soak;

static K_MUTEX_DEFINE(soak_lock);
static struct k_work_delayable soak_work;

static void soak_finish_locked(const char *reason)
{
	int err = heater_hw_force_off();

	soak.active = false;
	if (!soak.test_mode_was_on) {
		test_mode_set(false);
	}
	printk("Soak stopped: %s%s\n", reason,
	       err ? " (heater off FAILED - inspect hardware)" : "");
	LOG_INF("Soak stopped: %s err=%d", reason, err);
}

static void soak_tick(struct k_work *work)
{
	ARG_UNUSED(work);
	k_mutex_lock(&soak_lock, K_FOREVER);

	if (!soak.active) {
		k_mutex_unlock(&soak_lock);
		return;
	}

	int64_t now = k_uptime_get();
	if (now >= soak.deadline_ms) {
		soak_finish_locked("duration complete");
		k_mutex_unlock(&soak_lock);
		return;
	}

	if (!heater_power_ready() || !heater_external_power_present()) {
		soak_finish_locked("external power lost");
		k_mutex_unlock(&soak_lock);
		return;
	}

	struct sensor_temperature_observation observation;
	if (sensor_get_imu_temperature_observation(&observation, 2000) != 0 ||
	    !isfinite(observation.raw_c) || !isfinite(observation.filtered_c)) {
		soak_finish_locked("temperature observation stale");
		k_mutex_unlock(&soak_lock);
		return;
	}

	if (observation.raw_c >= SOAK_CEILING_C) {
		soak_finish_locked("temperature ceiling reached");
		k_mutex_unlock(&soak_lock);
		return;
	}

	if (!soak.closed_loop) {
		soak.duty = soak.max_duty;
	} else if (observation.raw_c < soak.target - SOAK_HYSTERESIS_C) {
		soak.duty = soak.max_duty;
	} else if (observation.raw_c > soak.target + SOAK_HYSTERESIS_C) {
		soak.duty = 0;
	}

	int err = heater_hw_write(soak.generation, soak.sequence++,
				  observation.sampled_at_ms, observation.raw_c, soak.duty);
	if (err) {
		soak_finish_locked("heater write failed");
		k_mutex_unlock(&soak_lock);
		return;
	}

	if (now >= soak.next_log_ms) {
		soak.next_log_ms = now + SOAK_LOGGING_MS;
		LOG_INF("Soak: target=%.1fC raw=%.2fC filtered=%.2fC duty=%u/%u remaining=%llds",
			(double)soak.target, (double)observation.raw_c,
			(double)observation.filtered_c, soak.duty, soak.max_duty,
			(long long)((soak.deadline_ms - now) / 1000));
	}

	k_work_reschedule(&soak_work, K_MSEC(SOAK_TICK_MS));
	k_mutex_unlock(&soak_lock);
}

static int soak_begin(uint16_t minutes, float target_c, uint16_t duty_pptt, bool closed_loop)
{
	if (minutes == 0 || minutes > 120) {
		return -EINVAL;
	}
	if (closed_loop &&
	    (!isfinite(target_c) || target_c < 20.0f || target_c >= SOAK_CEILING_C)) {
		return -EINVAL;
	}
	if (duty_pptt == 0 || duty_pptt > 9900) {
		return -EINVAL;
	}
	if (!heater_hw_available()) {
		return -ENODEV;
	}
	if (!heater_power_ready() || !heater_external_power_present()) {
		return -ENODEV;
	}
	/* Never compete with a calibration session for the sensor/heater. */
	if (sensor_tcal_heated_busy()) {
		return -EBUSY;
	}

	k_mutex_lock(&soak_lock, K_FOREVER);
	if (soak.active) {
		k_mutex_unlock(&soak_lock);
		return -EALREADY;
	}

	soak.generation++;
	if (soak.generation == 0) {
		soak.generation++;
	}
	int err = heater_hw_arm(soak.generation);
	if (err) {
		k_mutex_unlock(&soak_lock);
		return err;
	}

	int64_t now = k_uptime_get();
	soak.active = true;
	soak.closed_loop = closed_loop;
	soak.sequence = 0;
	soak.max_duty = duty_pptt;
	soak.duty = duty_pptt;
	soak.target = target_c;
	soak.started_ms = now;
	soak.deadline_ms = now + (int64_t)minutes * 60000;
	soak.next_log_ms = now;
	soak.test_mode_was_on = test_mode_get();
	test_mode_set(true);

	k_work_reschedule(&soak_work, K_MSEC(200));
	k_mutex_unlock(&soak_lock);

	if (closed_loop) {
		printk("Soak started: %u min, closed loop at %.1f C, duty cap %u/10000.\n",
		       minutes, (double)target_c, duty_pptt);
	} else {
		printk("Soak started: %u min, open loop at fixed duty %u/10000.\n",
		       minutes, duty_pptt);
	}
	printk("Ceiling %.0f C, abort on external power loss. 'soak stop' ends it early.\n",
	       (double)SOAK_CEILING_C);
	return 0;
}

int heater_soak_start(uint16_t minutes, float target_c)
{
	return soak_begin(minutes, target_c, CONFIG_SYSTEM_IMU_HEATER_MAX_DUTY_PPTT, true);
}

int heater_soak_start_raw(uint16_t minutes, uint16_t duty_pptt)
{
	return soak_begin(minutes, 0.0f, duty_pptt, false);
}

void heater_soak_stop(void)
{
	k_mutex_lock(&soak_lock, K_FOREVER);
	if (soak.active) {
		soak_finish_locked("user stop");
	} else {
		printk("Soak is not active.\n");
	}
	k_mutex_unlock(&soak_lock);
}

bool heater_soak_active(void)
{
	k_mutex_lock(&soak_lock, K_FOREVER);
	bool active = soak.active;
	k_mutex_unlock(&soak_lock);
	return active;
}

void heater_soak_report(void)
{
	k_mutex_lock(&soak_lock, K_FOREVER);
	bool active = soak.active;
	bool closed_loop = soak.closed_loop;
	float target = soak.target;
	uint16_t duty = soak.duty;
	uint16_t max_duty = soak.max_duty;
	int64_t remaining = soak.deadline_ms - k_uptime_get();
	k_mutex_unlock(&soak_lock);

	struct sensor_temperature_observation observation;
	bool fresh = sensor_get_imu_temperature_observation(&observation, 2000) == 0;

	if (closed_loop) {
		printk("Soak: %s closed loop target=%.1fC duty=%u/%u remaining=%llds temperature=",
		       active ? "active" : "inactive", (double)target, duty, max_duty,
		       (long long)(remaining / 1000));
	} else {
		printk("Soak: %s open loop duty=%u/%u remaining=%llds temperature=",
		       active ? "active" : "inactive", duty, max_duty,
		       (long long)(remaining / 1000));
	}
	if (fresh) {
		printk("raw=%.2fC filtered=%.2fC\n", (double)observation.raw_c,
		       (double)observation.filtered_c);
	} else {
		printk("unavailable/stale\n");
	}
}

#else /* !CONFIG_SENSOR_TCAL_HEATED */

int heater_soak_start(uint16_t minutes, float target_c)
{
	ARG_UNUSED(minutes);
	ARG_UNUSED(target_c);
	return -ENOTSUP;
}

int heater_soak_start_raw(uint16_t minutes, uint16_t duty_pptt)
{
	ARG_UNUSED(minutes);
	ARG_UNUSED(duty_pptt);
	return -ENOTSUP;
}

void heater_soak_stop(void)
{
}

bool heater_soak_active(void)
{
	return false;
}

void heater_soak_report(void)
{
	printk("Heater soak unavailable (CONFIG_SENSOR_TCAL_HEATED off).\n");
}

#endif
