#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>

#define SELF_SUSPEND 0

#define BUTTON_DEBOUNCE_DELAY_MS 250

static uint8_t dummy_flg;
static void dummy_thread_set(uint8_t button);
static void gpio_button_init(void);
static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pin_pos);
void dummy_thread(void);
K_THREAD_DEFINE(dummy_tid, 2048, dummy_thread, NULL, NULL, NULL, 7, 0, 0);

#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)

void dummy_thread(void)
{
	bool self_suspend = IS_ENABLED(SELF_SUSPEND);
	while (1) {

		uint32_t now_ms = k_uptime_get_32();
		k_sleep(K_MSEC(500));
		printk("%u.%03u : %s flag = %d\n", now_ms / 1000U, now_ms % 1000,
		       self_suspend ? "SUSPENDING" : "",
		       dummy_flg);
		if (self_suspend) {
			k_thread_suspend(dummy_tid);
		}
	}
}

void main(void)
{
	gpio_button_init();
}

static struct gpio_callback button_cb_data;
const struct device *dev_button;
void gpio_button_init(void)
{
	int ret;

	dev_button = device_get_binding(DT_GPIO_LABEL(SW0_NODE, gpios));
	if (dev_button == NULL) {
		printk("Error: didn't find %s device\n",
			DT_GPIO_LABEL(SW0_NODE, gpios));
		return;
	}

	ret = gpio_pin_configure(dev_button, DT_GPIO_PIN(SW0_NODE, gpios),
				 DT_GPIO_FLAGS(SW0_NODE, gpios) | GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d '%s'\n",
			ret, DT_GPIO_PIN(SW0_NODE, gpios), DT_GPIO_LABEL(SW0_NODE, gpios));
		return;
	}

	ret = gpio_pin_configure(dev_button, DT_GPIO_PIN(SW1_NODE, gpios),
				 DT_GPIO_FLAGS(SW1_NODE, gpios) | GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d '%s'\n",
			ret, DT_GPIO_PIN(SW1_NODE, gpios), DT_GPIO_LABEL(SW1_NODE, gpios));
		return;
	}

	ret = gpio_pin_interrupt_configure(dev_button, DT_GPIO_PIN(SW0_NODE, gpios),
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on pin %d '%s'\n",
			ret, DT_GPIO_PIN(SW0_NODE, gpios), DT_GPIO_LABEL(SW0_NODE, gpios));
		return;
	}

	ret = gpio_pin_interrupt_configure(dev_button, DT_GPIO_PIN(SW1_NODE, gpios),
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on pin %d '%s'\n",
			ret, DT_GPIO_PIN(SW1_NODE, gpios), DT_GPIO_LABEL(SW1_NODE, gpios));
		return;
	}
	gpio_init_callback(&button_cb_data, button_pressed,
			   BIT(DT_GPIO_PIN(SW0_NODE, gpios)) | BIT(DT_GPIO_PIN(SW1_NODE, gpios)));
	gpio_add_callback(dev_button, &button_cb_data);
}

static uint8_t pin_to_sw(uint32_t pin_pos)
{
	switch (pin_pos) {
	case BIT(DT_GPIO_PIN(SW0_NODE, gpios)): return 0;
	case BIT(DT_GPIO_PIN(SW1_NODE, gpios)): return 1;

	default:
		printk("Invalid pin_pos\n");
	}

	printk("No match for GPIO pin 0x%08x\n", pin_pos);

	return 0;
}

static uint32_t time, last_time;
static void button_pressed(const struct device *dev, struct gpio_callback *cb,
		uint32_t pin_pos)
{
	uint8_t button;

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

static void dummy_thread_set(uint8_t button)
{
	static uint8_t rule = 0;
	if (button) {
		++rule;
		if (rule & 0x01) {
			printk("R%u Wakeup dummy thread\n", rule & 0x03);
			k_wakeup(dummy_tid);
		}
		if (rule & 0x02) {
			printk("R%u Resume dummy thread\n", rule & 0x03);
			k_thread_resume(dummy_tid);
		}

	} else {
		printk("Suspend dummy thread\n");
		k_thread_suspend(dummy_tid);
	}
}
