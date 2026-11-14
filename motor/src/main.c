/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Motor co-sim demo for the QEMU stm32n6570-dk, mirroring the
 * FreeRTOS demo's test_motor + motor current-sense phases:
 *
 *   TIM2 CH2 (Zephyr PWM API) -> virtual motor drive input
 *   GPIOD 0/1                 -> driver DIR / EN lines
 *   TIM3 CH1/CH2 capture      -> tach wheel A/B edge counting
 *   ADC1 channel 0 (API)      -> motor current-sense code
 *
 * Run it with ~/qemu/run-n6-zephyr-motor.sh (QEMU boots this image
 * XIP through the XSPI2 window, boot-base=0x70000000).
 *
 * Hardware-port caveats (fine on the co-sim, wrong on silicon):
 *  - TIM3 is driven through raw registers (capture); app.overlay sets
 *    timers3 status="okay" so its kernel clock is enabled, but a real
 *    port also needs the tach inputs routed through pinctrl AF entries.
 *  - current sense uses ADC1 channel 0 (AIN0) because the QEMU ADC
 *    model feeds the motor isen there; the board pinctrl maps ADC1 to
 *    channels 10/11, so silicon would need channel-0 pin routing.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>


/* the HAL header defines TIM3_BASE at the non-secure alias; use the
 * DT (secure-view) address and interrupt for this app */
#define T3_BASE  DT_REG_ADDR(DT_NODELABEL(timers3))
#define TIM3_IRQ DT_IRQ_BY_NAME(DT_NODELABEL(timers3), global, irq)

static volatile uint32_t mot_a_edges;
static volatile uint32_t mot_b_edges;
static volatile uint32_t mot_last_a;
static volatile uint32_t mot_ab_sum;
static volatile uint32_t mot_ab_n;

static inline uint32_t t3_read(unsigned int off)
{
	return *(volatile uint32_t *)(uintptr_t)(T3_BASE + off);
}

static inline void t3_write(unsigned int off, uint32_t val)
{
	*(volatile uint32_t *)(uintptr_t)(T3_BASE + off) = val;
}

#define T3_CR1   0x00
#define T3_DIER  0x0C
#define T3_SR    0x10
#define T3_EGR   0x14
#define T3_CCMR1 0x18
#define T3_CCER  0x20
#define T3_CCR1  0x34
#define T3_CCR2  0x38
#define T3_PSC   0x28
#define T3_ARR   0x2C

#define T3_CC1IE BIT(1)
#define T3_CC2IE BIT(2)
#define T3_CC1IF BIT(1)
#define T3_CC2IF BIT(2)

static void tim3_isr(void *arg)
{
	uint32_t sr = t3_read(T3_SR);

	if (sr & T3_CC1IF) { /* tach A */
		mot_last_a = t3_read(T3_CCR1);
		mot_a_edges++;
	}
	if (sr & T3_CC2IF) { /* tach B, lags A while driving */
		int32_t d = (int32_t)(t3_read(T3_CCR2) - mot_last_a);

		if ((d > 0) && (d < 84)) {
			mot_ab_sum += d;
			mot_ab_n++;
		}
		mot_b_edges++;
	}
	t3_write(T3_SR, ~(sr & (T3_CC1IF | T3_CC2IF))); /* write-0-to-clear */
	ARG_UNUSED(arg);
}

static void tim3_capture_start(void)
{
	t3_write(T3_CR1, 0);
	t3_write(T3_DIER, 0);
	t3_write(T3_SR, 0);
	t3_write(T3_PSC, 239);                 /* 100 kHz on the 24 MHz clk */
	t3_write(T3_ARR, 0xFFFF);
	t3_write(T3_CCMR1, BIT(0) | BIT(8));   /* CC1S=TI1, CC2S=TI2 */
	t3_write(T3_CCER, BIT(0) | BIT(4));    /* CC1E | CC2E */
	t3_write(T3_EGR, BIT(0));              /* UG */
	t3_write(T3_SR, 0);

	mot_a_edges = 0;
	mot_b_edges = 0;
	mot_ab_sum = 0;
	mot_ab_n = 0;
	IRQ_CONNECT(TIM3_IRQ, 3, tim3_isr, NULL, 0);
	irq_enable(TIM3_IRQ);
	t3_write(T3_DIER, T3_CC1IE | T3_CC2IE);
	t3_write(T3_CR1, BIT(0));              /* CEN */
}

static void capture_clear(void)
{
	mot_a_edges = 0;
	mot_b_edges = 0;
	mot_ab_sum = 0;
	mot_ab_n = 0;
}

static const struct device *const pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm2));
static const struct device *const adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));
static const uint32_t period_ns = 1000000; /* 1 kHz */
static uint32_t adc_buf[1]; /* N6 adc_data_size_t is uint32_t */

static uint32_t adc_sense(void)
{
	struct adc_sequence sequence = {
		.channels = BIT(0),
		.buffer = adc_buf,
		.buffer_size = sizeof(adc_buf),
		.resolution = 12,
	};

	int err = adc_read(adc_dev, &sequence);

	if (err != 0) {
		printk("adc read failed (%d)\n", err);
		return 0xDEAD;
	}
	return adc_buf[0];
}

static int pwm_drive(uint32_t duty_ns)
{
	/* demo-scale geometry: period 1000 counts, one count per
	 * microsecond of pulse; the model counts on its own TIM2
	 * clock, so ns-scale periods just scale the carrier */
	return pwm_set_cycles(pwm_dev, 2 /* CH2 */, 1000, duty_ns / 1000, 0);
}

int main(void)
{
	struct gpio_dt_spec dir = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_dir), gpios);
	struct gpio_dt_spec en = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_en), gpios);
	struct adc_channel_cfg chcfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		/* N6 DT sampling-times table is in ADC-clock ticks */
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 2),
		.channel_id = 0,
	};
	int pass = 1;
	int ret;
	uint32_t a, mean, coast, rest, v;


	printk("N6 motor co-sim (Zephyr)\n");

	if (!device_is_ready(pwm_dev) || !device_is_ready(adc_dev)) {
		printk("pwm or adc not ready\n");
		return 1;
	}
	if (!device_is_ready(dir.port) || !device_is_ready(en.port)) {
		printk("gpiod not ready\n");
		return 1;
	}

	gpio_pin_configure_dt(&dir, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&en, GPIO_OUTPUT_INACTIVE);

	ret = adc_channel_setup(adc_dev, &chcfg);
	if (ret != 0) {
		printk("adc channel setup failed (%d)\n", ret);
		return 1;
	}

	ret = pwm_drive(period_ns / 2);
	if (ret != 0) {
		printk("pwm set failed (%d)\n", ret);
		return 1;
	}

	tim3_capture_start();

	/* 1. disabled driver: PWM runs, wheel must not turn. */
	k_msleep(100);
	a = mot_a_edges;
	if (a != 0) {
		pass = 0;
	}
	printk("MOTOR idle a=%u %s\n", a, (a == 0) ? "PASS" : "FAIL");

	/* 2. run forward: tau 50 ms, count the settled last 100 ms. */
	gpio_pin_set_dt(&en, 1);
	k_msleep(150);
	capture_clear();
	k_msleep(100);
	a = mot_a_edges;
	mean = (mot_ab_n != 0) ? (mot_ab_sum / mot_ab_n) : 0;
	if ((a < 100) || (a > 420) || (mot_ab_n < 10) ||
	    (mean < 40) || (mean > 65)) {
		pass = 0;
	}
	printk("MOTOR fwd a=%u b=%u ab=%u mean=%u\n", a, mot_b_edges,
		mot_ab_n, mean);

	/* 3. coast to rest: decay, then silence. */
	gpio_pin_set_dt(&en, 0);
	capture_clear();
	k_msleep(300);
	coast = mot_a_edges;
	k_msleep(500);
	rest = mot_a_edges - coast;
	if (rest > 3) {
		pass = 0;
	}
	printk("MOTOR coast=%u rest=%u\n", coast, rest);

	/* 4. reverse. */
	gpio_pin_set_dt(&dir, 0);
	gpio_pin_set_dt(&en, 1);
	k_msleep(150);
	capture_clear();
	k_msleep(100);
	a = mot_a_edges;
	if ((a < 100) || (a > 420)) {
		pass = 0;
	}
	printk("MOTOR rev a=%u MOTOR %s\n", a, pass ? "PASS" : "FAIL");
	gpio_pin_set_dt(&en, 0);
	k_msleep(600); /* coast to rest before the current phases */

	/* 5. current sense: full-duty start saturates, no-load running
	 * relaxes to the friction floor, coasting reads exactly zero. */
	ret = pwm_drive(period_ns);
	if (ret != 0) {
		printk("pwm full duty failed (%d)\n", ret);
		return 1;
	}
	gpio_pin_set_dt(&dir, 1);
	gpio_pin_set_dt(&en, 1);
	v = adc_sense();
	if (v < 1500) {
		pass = 0;
	}
	printk("ADC start code=%u\n", v);
	k_msleep(500); /* settled no-load speed: slip current relaxes */
	v = adc_sense();
	if ((v < 150) || (v > 350)) {
		pass = 0;
	}
	printk("ADC run code=%u\n", v);
	gpio_pin_set_dt(&en, 0);
	k_msleep(50); /* let the 1 ms motor isen sampler relax */
	v = adc_sense();
	if (v != 0) {
		pass = 0;
	}
	printk("ADC stop code=%u ADC %s\n", v, pass ? "PASS" : "FAIL");

	return 0;
}
