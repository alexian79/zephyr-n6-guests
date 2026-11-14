/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr port of the sheet-transport co-sim controller (the FreeRTOS
 * transport-n6 demo). Runs on the QEMU stm32n6570-dk machine:
 *
 *   TIM2 CH1 (Zephyr PWM API) -> M1 domain, TIM5 CH1 -> M2 domain
 *   GPIOD 0/1/2/3             -> DIR1/DIR2/EN1/EN2 (statically driven)
 *   GPIOD 4/5/6/7             -> IN / S1 / S2 (active high), EXIT (active low)
 *   usart1 (chosen console)   -> event log, same format as transport-n6
 *
 * Run with ~/qemu/run-n6-zephyr-transport.sh (XIP through the XSPI2
 * window, boot-base=0x70000000, transport=TIM2:1;TIM5:1;D0;D1;D2;D3;D4;D5;D6;D7).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

#define LOG_LEVEL LOG_LEVEL_INF
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(transport);

static const struct device *const pwm_m1 = DEVICE_DT_GET(DT_NODELABEL(pwm2));
static const struct device *const pwm_m2 = DEVICE_DT_GET(DT_NODELABEL(pwm5));

static const struct gpio_dt_spec dir1 = GPIO_DT_SPEC_GET(DT_NODELABEL(dir1), gpios);
static const struct gpio_dt_spec dir2 = GPIO_DT_SPEC_GET(DT_NODELABEL(dir2), gpios);
static const struct gpio_dt_spec en1  = GPIO_DT_SPEC_GET(DT_NODELABEL(en1), gpios);
static const struct gpio_dt_spec en2  = GPIO_DT_SPEC_GET(DT_NODELABEL(en2), gpios);

static const struct gpio_dt_spec sen_in   = GPIO_DT_SPEC_GET(DT_NODELABEL(sen_in), gpios);
static const struct gpio_dt_spec sen_s1   = GPIO_DT_SPEC_GET(DT_NODELABEL(sen_s1), gpios);
static const struct gpio_dt_spec sen_s2   = GPIO_DT_SPEC_GET(DT_NODELABEL(sen_s2), gpios);
static const struct gpio_dt_spec sen_exit = GPIO_DT_SPEC_GET(DT_NODELABEL(sen_exit), gpios);

/* 1 kHz PWM, 350000 ns pulse = 350 permille ~420 mm/s surface speed
 * (identical duty to the FreeRTOS reference: 350/1000 of the period) */
#define PWM_PERIOD_NS 1000000ULL
#define DUTY_FEED_NS  350000ULL

/* Multi-note pipelining: point sensors cannot pair events to specific
 * notes, but every note passes IN -> S1 -> S2 -> EXIT in strict order,
 * so rising-edge counts per sensor identify how many notes occupy each
 * zone; the batch re-arms when every entered note has left.
 * (Same algorithm as transport/main.c.) */
static uint32_t nIn, nS1, nS2, nOut;   /* rising-edge counters */
static bool pIn, pS1, pS2, pEx;        /* previous sensor levels */
static int mM1, mM2;                   /* latched pwm state */
static bool seeded;

static void set_pwm(const struct device *pwm, bool on)
{
	/* stm32 PWM driver channels are 1-based: CH1 -> 1 */
	int err = pwm_set(pwm, 1, PWM_PERIOD_NS, on ? DUTY_FEED_NS : 0, 0);

	if (err) {
		LOG_ERR("pwm set failed: %d", err);
	}
}

static void log_msg(const char *evt)
{
	LOG_INF("[%4lld ms] %-9s in=%u out=%u state=%d%d",
		k_uptime_get(), evt, (unsigned int)nIn,
		(unsigned int)nOut, mM1, mM2);
}

static int read_sen(const struct gpio_dt_spec *sen)
{
	static bool err_logged;
	int v = gpio_pin_get_dt(sen);

	if (v < 0) {
		/* driver error: treat as unblocked, but say so once */
		if (!err_logged) {
			err_logged = true;
			LOG_ERR("sensor read failed: %d", v);
		}
		return 0;
	}
	return v;
}

static void transport_loop(void)
{
	int in = read_sen(&sen_in);
	int s1 = read_sen(&sen_s1);
	int s2 = read_sen(&sen_s2);
	int ex = read_sen(&sen_exit);
	int wantM1, wantM2;

	if (!seeded) {
		/* a note already on the beams at start counts as entered */
		seeded = true;
		pIn = in != 0;
		pS1 = s1 != 0;
		pS2 = s2 != 0;
		pEx = ex != 0;
		nIn = pIn ? 1U : 0U;
		nS1 = pS1 ? 1U : 0U;
		nS2 = pS2 ? 1U : 0U;
		nOut = 0U;
	}

	if (in && !pIn) {
		nIn++;
		log_msg("IN blk");
	}
	if (s1 && !pS1) {
		nS1++;
		log_msg("S1 blk");
	}
	if (s2 && !pS2) {
		nS2++;
		log_msg("S2 blk");
	}
	if (ex && !pEx) {
		log_msg("EXIT blk");
	}
	if (pEx && !ex) {
		nOut++;
		log_msg("EXIT clr");
	}
	pIn = in != 0;
	pS1 = s1 != 0;
	pS2 = s2 != 0;
	pEx = ex != 0;

	/* drive demand from zone occupancy, not a single-note state */
	wantM1 = in || (nS1 > nS2);
	wantM2 = nS1 > nOut;

	if (wantM1 != mM1) {
		mM1 = wantM1;
		set_pwm(pwm_m1, mM1 != 0);
		log_msg(mM1 ? "M1 drive" : "M1 off");
	}
	if (wantM2 != mM2) {
		mM2 = wantM2;
		set_pwm(pwm_m2, mM2 != 0);
		log_msg(mM2 ? "M2 drive" : "M2 off");
	}

	if ((nOut > 0U) && (nOut == nIn)) {
		/* every entered note has left the bed: re-arm the batch */
		LOG_INF("[%4lld ms] COMPLETE  transport-zephyr notes=%u",
			k_uptime_get(), (unsigned int)nIn);
		nIn = nS1 = nS2 = nOut = 0U;
	}
}

int main(void)
{
	static const struct gpio_dt_spec *const pins[] = {
		&dir1, &dir2, &en1, &en2, &sen_in, &sen_s1, &sen_s2, &sen_exit
	};
	static const gpio_flags_t flags[] = {
		GPIO_OUTPUT_ACTIVE, GPIO_OUTPUT_ACTIVE,
		GPIO_OUTPUT_ACTIVE, GPIO_OUTPUT_ACTIVE,
		GPIO_INPUT, GPIO_INPUT, GPIO_INPUT, GPIO_INPUT
	};

	for (int i = 0; i < ARRAY_SIZE(pins); i++) {
		int err = gpio_pin_configure_dt(pins[i], flags[i]);

		if (err) {
			LOG_ERR("gpio %d cfg failed: %d", i, err);
			return -EIO;
		}
	}

	if (!device_is_ready(pwm_m1) || !device_is_ready(pwm_m2)) {
		LOG_ERR("pwm not ready");
		return -ENODEV;
	}
	set_pwm(pwm_m1, false);
	set_pwm(pwm_m2, false);

	LOG_INF("transport-zephyr: sheet controller ready");

	for (;;) {
		transport_loop();
		k_msleep(10);
	}
	return 0;
}
