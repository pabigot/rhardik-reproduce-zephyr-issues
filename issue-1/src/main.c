#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>

#define BUTTON_DEBOUNCE_DELAY_MS 250

static u8_t dummy_flg;
static void dummy_thread_set(u8_t button);
static void gpio_button_init(void);
static void button_pressed(struct device *dev, struct gpio_callback *cb, u32_t pin_pos);
void dummy_thread(void);
K_THREAD_DEFINE(dummy_tid, 2048, dummy_thread, NULL, NULL, NULL, 7, 0, K_NO_WAIT);

void dummy_thread(void)
{
	while (1) {
			k_sleep(K_MSEC(8000));
			printk("flag = %d\n",dummy_flg);
			printk("%s\n", __func__);
			//k_thread_suspend(dummy_tid);
	}
}

void main(void)
{
	gpio_button_init();
}

static struct gpio_callback button_cb_data;
struct device *dev_button;
void gpio_button_init(void)
{
	int ret;

	dev_button = device_get_binding(DT_ALIAS_SW0_GPIOS_CONTROLLER);
	if (dev_button == NULL) {
		printk("Error: didn't find %s device\n",
			DT_ALIAS_SW0_GPIOS_CONTROLLER);
		return;
	}

	ret = gpio_pin_configure(dev_button, DT_ALIAS_SW0_GPIOS_PIN,
				 DT_ALIAS_SW0_GPIOS_FLAGS | GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d '%s'\n",
			ret, DT_ALIAS_SW0_GPIOS_PIN, DT_ALIAS_SW0_LABEL);
		return;
	}

	ret = gpio_pin_configure(dev_button, DT_ALIAS_SW1_GPIOS_PIN,
				 DT_ALIAS_SW1_GPIOS_FLAGS | GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d '%s'\n",
			ret, DT_ALIAS_SW1_GPIOS_PIN, DT_ALIAS_SW1_LABEL);
		return;
	}

	ret = gpio_pin_interrupt_configure(dev_button, DT_ALIAS_SW0_GPIOS_PIN,
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on pin %d '%s'\n",
			ret, DT_ALIAS_SW0_GPIOS_PIN, DT_ALIAS_SW0_LABEL);
		return;
	}

	ret = gpio_pin_interrupt_configure(dev_button, DT_ALIAS_SW1_GPIOS_PIN,
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on pin %d '%s'\n",
			ret, DT_ALIAS_SW1_GPIOS_PIN, DT_ALIAS_SW1_LABEL);
		return;
	}
	gpio_init_callback(&button_cb_data, button_pressed,
			   BIT(DT_ALIAS_SW0_GPIOS_PIN) | BIT(DT_ALIAS_SW1_GPIOS_PIN));
	gpio_add_callback(dev_button, &button_cb_data);


}

static uint8_t pin_to_sw(uint32_t pin_pos)
{
	switch (pin_pos) {
	case BIT(DT_GPIO_KEYS_SW0_GPIOS_PIN): return 0;
	case BIT(DT_GPIO_KEYS_SW1_GPIOS_PIN): return 1;

	default:
		printk("Invalid pin_pos\n");
	}

	printk("No match for GPIO pin 0x%08x\n", pin_pos);

	return 0;
}

static u32_t time, last_time;
static void button_pressed(struct device *dev, struct gpio_callback *cb,
		u32_t pin_pos)
{
	u8_t button;

	time = k_uptime_get_32();

	/* debounce the switch */
	if (time < last_time + BUTTON_DEBOUNCE_DELAY_MS) {
		last_time = time;
		return;
	}

	last_time = time;

	button = pin_to_sw(pin_pos);

	dummy_thread_set(button);
}

static void dummy_thread_set(u8_t button)
{
	if (button) {

		printk("Wakeup dummy thread\n");
		k_wakeup(dummy_tid);

		printk("Resume dummy thread\n");
		k_thread_resume(dummy_tid);
	} else {
		printk("Suspend dummy thread\n");
		k_thread_suspend(dummy_tid);
	}
}
