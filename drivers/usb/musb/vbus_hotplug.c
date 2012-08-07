/*
 * Ingenic USB Device Controller Hotplug Driver.
 *
 * Author: River Wang <zwang@ingenic.cn>
 */

/*
How to use

1. Basic usage:
All controlling interfaces are locacated in /sys/devices/platform/jz4740_udc/.
[uh_cable]: It indicates the status of the cable: offline/power/usb.

2. Asychronous notification:
You can use a non-blocking PF_NETLINK socket to receive the uevent sent by udc_houplug. DRIVER & UDC_HOTPLUG_CABLE_STATE varible in uevent can be used to get the status of UDC cable.

3. Notification mode & Gadget loading:
There are two notification modes can be configured by [uh_notify_mode]: auto/manual.
Auto: When a USB cable with active signals is plugged in, the udc_hotplug driver will broadcast this event to jz4740_udc & userspace app, and jz4740_udc will try to activate the current gadget.

This mode is used when userspace APP only wants to get the cable status notification.

Manual: The event will only be broadcast to userspace APP. When APP decides to how to handle this event, It can make it with [uh_notify]:
auto: The udc_hotplug driver broadcast the latest cable status to jz4740_udc.
offline/power/usb:....

This mode is recommended when multiple gadgets are used. Userspace APP is notified by the uevent, load the specific gadget module by insmod, then set [uh_notify] to broadcat the event at last.
*/

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/kobject.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>

#include <asm/mach-jz4770/board-gcw0.h>
#include <asm/mach-jz4770/jz4770cpm.h>
#include <asm/mach-jz4770/jz4770gpio.h>
#include <asm/mach-jz4770/jz4770intc.h>
#include <asm/mach-jz4770/jz4770otg.h>


#define D(msg,   fmt...)
//#define D(msg, fmt...)  \
	printk(KERN_ERR JZ_VH_PFX": %s(): "msg, __func__, ##fmt)

#define JZ_VH_PFX "jz_vbus_hotplug"

#define NR_UDC_WAIT_INTR_LOOP	(5 * 1000 * 1000)

#define DEFAULT_KEEP_ALIVE_TIMER_INTERVAL     (1 * HZ)
#define DEFAULT_KEEP_ALIVE_COUNTER_LIMIT      2


typedef enum {
	UH_NOTIFY_CABLE_STATE = 0,
}uh_notify_type_t;

typedef enum {
	UH_CABLE_STATE_OFFLINE = 0,
	UH_CABLE_STATE_POWER,
	UH_CABLE_STATE_USB,
}uh_cable_state_t;

typedef enum {
	UH_THREAD_STATE_IDLE = 0,
	UH_THREAD_STATE_START,
	UH_THREAD_STATE_BUSY,
	UH_THREAD_STATE_DONE,
}uh_thread_state_t;

/* UDC Flag bits */
enum {
	BIT_UH_ENABLE,

	/* State changed ?*/
	BIT_CABLE_STATE_CHANGE,

	BIT_DO_DETECT,
	BIT_DO_NOTIFY,

	/* Keep alive */
	BIT_KEEP_ALIVE,
	BIT_KEEP_ALIVE_STOP,
};

#define DO_DETECT (1 << BIT_DO_DETECT)
#define DO_NOTIFY (1 << BIT_DO_NOTIFY)
#define DO_ALL ( DO_DETECT | DO_NOTIFY)

struct uh_data {
	unsigned long flags;

	/* Notifier */
	struct blocking_notifier_head notifier_head;

	/* Thread */
	struct task_struct *kthread;

	int b_notify_mode;

	/* Wait queue */
	wait_queue_head_t kthread_wq;	/* Kernel thread sleep here. */
	wait_queue_head_t timer_wq;	/* Wake up when timer is finished. */
	wait_queue_head_t finish_wq;	/* Wake up when thread is finished. */

	uh_thread_state_t thread_state;
	uh_cable_state_t cable_state;
	uh_cable_state_t cable_detect_state;

	struct timer_list stable_timer;
	struct timer_list keep_alive_timer; /* Keep alive */

	unsigned long keep_alive_counter_limit;
	unsigned long keep_alive_timer_interval;
	unsigned long keep_alive_counter;

	int gpio_irq;
	int gpio_pin;

	struct platform_device *pdev;
};

static struct uh_data *g_puh_data = NULL;

static inline void uh_start_work(struct uh_data *uh, int work)
{
	if (work & DO_DETECT) {
		set_bit(BIT_DO_DETECT, &uh->flags);
	}

	if (work & DO_NOTIFY) {
		set_bit(BIT_DO_NOTIFY, &uh->flags);
	}

	mod_timer(&uh->stable_timer, 1 + jiffies);

	return;
}

static inline void set_cable_state(struct uh_data *uh, uh_cable_state_t state)
{
	if (uh->cable_state != state) {
		D("Cable state: %d -> %d.\n", uh->cable_state, state);

		uh->cable_state = state;
		set_bit(BIT_CABLE_STATE_CHANGE, &uh->flags);
	}

	return;
}

static void uh_stable_timer_func(unsigned long data)
{
	struct uh_data *uh = (struct uh_data *)data;

	D("Called.\n");

	if (!test_bit(BIT_UH_ENABLE, &uh->flags))
		return;

	uh->thread_state = UH_THREAD_STATE_START;

	/* Start. */
	wake_up_process(uh->kthread);

	return;
}

/* Do cable detection */
static void cable_detect(struct uh_data *uh)
{
	if (__gpio_get_pin(uh->gpio_pin)) {
		D("Cable online.\n");

		uh->cable_detect_state = UH_CABLE_STATE_POWER;

	}else {
		D("Cable offline.\n");

		clear_bit(BIT_KEEP_ALIVE, &uh->flags);

		uh->cable_detect_state = UH_CABLE_STATE_OFFLINE;
	}

	return;
}

/* USB is active ? */
static int usb_is_active(void)
{
	unsigned volatile long timeout = NR_UDC_WAIT_INTR_LOOP;
	unsigned long frame_no = REG16(USB_REG_FRAME);

	/*
	   Some power charger may cause fake SOF,
	   We must handle this situation.
					- River.
	*/

	int counter = 7;

	while (timeout && counter) {
		if (frame_no != REG16(USB_REG_FRAME)) {
			if (!--counter)
				break;

			/* Wait next frame. */
			frame_no = REG16(USB_REG_FRAME);
		}

		timeout --;
	}

	D("timout: %lu, counter: %d.\n", timeout, counter);

	return timeout ? 1 : 0;
}

/* Really do USB detection */
static int do_usb_detect(struct uh_data *uh)
{
	int rv;

	D("Called.\n");

	__intc_mask_irq(IRQ_OTG);

	/* Now enable PHY to start detect */
	__cpm_enable_otg_phy();

	/* Clear IRQs */
	REG16(USB_REG_INTRINE) = 0;
	REG16(USB_REG_INTROUTE) = 0;
	REG8(USB_REG_INTRUSBE) = 0;

	/* disable UDC IRQs first */
	REG16(USB_REG_INTRINE) = 0;
	REG16(USB_REG_INTROUTE) = 0;
	REG8(USB_REG_INTRUSBE) = 0;

	/* Disable DMA */
	REG32(USB_REG_CNTL1) = 0;
	REG32(USB_REG_CNTL2) = 0;

	/* Enable HS Mode */
	REG8(USB_REG_POWER) |= USB_POWER_HSENAB;
	/* Enable soft connect */
	REG8(USB_REG_POWER) |= USB_POWER_SOFTCONN;

	rv = usb_is_active();

	/* Detect finish ,clean every thing */
	/* Disconnect from usb */
	REG8(USB_REG_POWER) &= ~USB_POWER_SOFTCONN;

	/* Disable the USB PHY */
	__cpm_suspend_otg_phy();

	/* Clear IRQs */
	REG16(USB_REG_INTRINE) = 0;
	REG16(USB_REG_INTROUTE) = 0;
	REG8(USB_REG_INTRUSBE) = 0;

	__intc_ack_irq(IRQ_OTG);
	__intc_unmask_irq(IRQ_OTG);

	return rv;
}

/* Do USB bus protocol detection */
static void usb_detect(struct uh_data *uh)
{
	int rv = 0;

	D("Called.\n");

	/* If the cable has already been offline, we just pass the real USB detection. */
	if (uh->cable_detect_state != UH_CABLE_STATE_OFFLINE) {
		D("Do real detection.\n");
		rv = do_usb_detect(uh);
	}else{
		D("No need to do real detection.\n");
	}

	if (rv) {
		/* Online. */
		uh->cable_detect_state = UH_CABLE_STATE_USB;
	}else{
		/* No USB Signal. */
		if (uh->cable_detect_state == UH_CABLE_STATE_POWER) {
			/* TODO: Wait USB alive again. */
		}
	}

	return;
}

static void do_wait(struct uh_data *uh)
{
	D("Called.\n");

	wait_event(uh->kthread_wq, uh->thread_state == UH_THREAD_STATE_START);

	uh->thread_state = UH_THREAD_STATE_BUSY;

	return;
}

/* Called from kernel thread */
static void do_detect(struct uh_data *uh)
{
	D("Called.\n");

	if (!test_and_clear_bit(BIT_DO_DETECT, &uh->flags))
		return;

	D("Do detect.\n");

	if(__gpio_get_pin(uh->gpio_pin)) {
		cable_detect(uh);
		usb_detect(uh);

		set_cable_state(uh, uh->cable_detect_state);
	}else{
		set_cable_state(uh, UH_CABLE_STATE_OFFLINE);
	}

	return;
}

static void __do_notify(struct uh_data *uh)
{
	D("Called.\n");

	if (test_and_clear_bit(BIT_CABLE_STATE_CHANGE, &uh->flags)) {
		D("Kick notifier chain.\n");

		blocking_notifier_call_chain(&uh->notifier_head,
			UH_NOTIFY_CABLE_STATE, &uh->cable_state);


		D("Send uevent to userspace.\n");

		switch (uh->cable_state) {
			case UH_CABLE_STATE_USB:
				kobject_uevent(&uh->pdev->dev.kobj, KOBJ_ADD);
				break;

			case UH_CABLE_STATE_POWER:
				kobject_uevent(&uh->pdev->dev.kobj, KOBJ_CHANGE);
				break;

			case UH_CABLE_STATE_OFFLINE:
				kobject_uevent(&uh->pdev->dev.kobj, KOBJ_REMOVE);
				break;
		}
	}

	return;
}

static inline void do_notify(struct uh_data *uh)
{
	if (!uh->b_notify_mode)	/* Auto nofity mode. */
		set_bit(BIT_DO_NOTIFY, &uh->flags);

	if (test_and_clear_bit(BIT_DO_NOTIFY, &uh->flags))
		__do_notify(uh);

	return;
}

static inline void do_done(struct uh_data *uh)
{
	D("Done.\n");

	uh->thread_state = UH_THREAD_STATE_IDLE;

	wake_up(&uh->finish_wq);

	return;
}

/* Kernel thread */
static int uh_thread(void *data)
{
	struct uh_data *uh = (struct uh_data *)data;

	while (!kthread_should_stop()) {
		do_wait(uh);

		if (kthread_should_stop())
			break;

		do_detect(uh);

		do_notify(uh);

		do_done(uh);
	}

	D("Exit.\n");

	return 0;
}

static irqreturn_t uh_irq(int irq, void *dev_id)
{
	struct uh_data *uh = (struct uh_data *)dev_id;

	D("Called.\n");

	uh_start_work(uh, DO_DETECT);

	return IRQ_HANDLED;
}

static void uh_init_gpio(struct uh_data *uh)
{
        /* get current pin level */
	__gpio_disable_pull(uh->gpio_pin);
        __gpio_as_input(uh->gpio_pin);

	udelay(1);

	/* Because of every plug IN/OUT action will casue more than one interrupt,
	   So whether rising trigger or falling trigger method can both start the detection.
         */

	__gpio_as_irq_rise_edge(uh->gpio_pin);

	return;
}

static void uh_keep_alive_timer_func(unsigned long data)
{
	struct uh_data *uh = (struct uh_data *)data;

//	D("Timer running.\n");

	/* Decrease the counter. */
	if (test_bit(BIT_KEEP_ALIVE, &uh->flags)
			&& !(--uh->keep_alive_counter)) {

		if (!usb_is_active()) {
			D("Timeout.\n");

			clear_bit(BIT_KEEP_ALIVE, &uh->flags);

			if (uh->cable_state == UH_CABLE_STATE_USB)
				set_cable_state(uh, UH_CABLE_STATE_POWER);

			uh_start_work(uh, DO_NOTIFY);
		}
	}

	/* Set next active time. */
	if (test_bit(BIT_KEEP_ALIVE, &uh->flags)) {
		mod_timer(&uh->keep_alive_timer, uh->keep_alive_timer_interval + jiffies);
	}else{
		D("Timer will stop.\n");

		set_bit(BIT_KEEP_ALIVE_STOP, &uh->flags);
		wake_up(&uh->timer_wq);
	}

	return;
}

static void uh_set_counter(unsigned long timer_interval_in_jiffies, unsigned long counter_limit)
{
	struct uh_data *uh = g_puh_data;

	uh->keep_alive_timer_interval = timer_interval_in_jiffies;
	uh->keep_alive_counter_limit = counter_limit;

	uh->keep_alive_counter = uh->keep_alive_counter_limit;

	return;
}

static void uh_disable(void)
{
	struct uh_data *uh = g_puh_data;

	/* Disable the source of input. */
	clear_bit(BIT_UH_ENABLE, &uh->flags);

	if (test_and_clear_bit(BIT_KEEP_ALIVE, &uh->flags))
		/* Wait timer stop. */
		wait_event(uh->timer_wq, test_and_clear_bit(BIT_KEEP_ALIVE_STOP, &uh->flags));

	/* Wait thread idle. */
	wait_event(uh->finish_wq, uh->thread_state == UH_THREAD_STATE_IDLE);

	return;
}

static void uh_enable(void)
{
	struct uh_data *uh = g_puh_data;

	set_bit(BIT_UH_ENABLE, &uh->flags);

	return;
}

void uh_alive(void)
{
	struct uh_data *uh = g_puh_data;

	if (!test_bit(BIT_UH_ENABLE, &uh->flags))
		return;

        /* Reset counter */
	uh->keep_alive_counter = uh->keep_alive_counter_limit;

	/* We are alive. */
	if (!test_bit(BIT_KEEP_ALIVE, &uh->flags)) {
		D("Active timer.\n");

		/* Active timer. */
		set_bit(BIT_KEEP_ALIVE, &uh->flags);
		clear_bit(BIT_KEEP_ALIVE_STOP, &uh->flags);

		mod_timer(&uh->keep_alive_timer, 3 + jiffies);
	}

	return;
}
EXPORT_SYMBOL(uh_alive);

int uh_register_notifier(struct notifier_block *n)
{
	struct uh_data *uh = g_puh_data;

	D("Register notifier: 0x%p.\n", (void *)n);

	BUG_ON(!n->notifier_call);

	/* Notify in auto mode. */
	if (!uh->b_notify_mode)
		n->notifier_call(n, UH_NOTIFY_CABLE_STATE, &uh->cable_state);

	return blocking_notifier_chain_register(&uh->notifier_head, n);
}EXPORT_SYMBOL(uh_register_notifier);

int uh_unregister_notifier(struct notifier_block *n)
{
	struct uh_data *uh = g_puh_data;

	int rv;

	D("Unregister notifier: 0x%p.\n", (void *)n);

	/* Wait all stop. */
	uh_disable();

	rv = blocking_notifier_chain_unregister(&uh->notifier_head, n);

	uh_enable();

	uh_start_work(uh, DO_DETECT);	/* Update status. */

	return rv;
}EXPORT_SYMBOL(uh_unregister_notifier);

static ssize_t show_cable(struct device *device, struct device_attribute *attr,
			char *buf)
{
	struct uh_data *uh = g_puh_data;

	char *s = NULL;

	switch (uh->cable_state) {
		case UH_CABLE_STATE_OFFLINE:
			s = "offline";
			break;

		case UH_CABLE_STATE_POWER:
			s = "power";
			break;

		case UH_CABLE_STATE_USB:
			s = "usb";
			break;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", s);
}

static ssize_t show_notify_mode(struct device *device, struct device_attribute *attr,
			char *buf)
{
	struct uh_data *uh = g_puh_data;

	char *s = uh->b_notify_mode ? "manual" : "auto";

	return snprintf(buf, PAGE_SIZE, "%s\n", s);
}

static ssize_t store_notify_mode(struct device *device, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct uh_data *uh = g_puh_data;

	if (!strncmp(buf, "auto", 4)) {
		uh->b_notify_mode = 0;
	}else if (!strncmp(buf, "manual", 6)) {
		uh->b_notify_mode = 1;
	}

	return count;
}


static ssize_t store_notify(struct device *device, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct uh_data *uh = g_puh_data;

	if (!strncmp(buf, "auto", 4)) {
		uh_start_work(uh, DO_ALL);
	}

	return count;
}

static struct device_attribute uh_sysfs_attrs[] = {
	__ATTR(uh_cable, S_IRUGO|S_IWUSR, show_cable, NULL),
	__ATTR(uh_notify_mode, S_IRUGO|S_IWUSR, show_notify_mode, store_notify_mode),
	__ATTR(uh_notify, S_IRUGO|S_IWUSR, NULL, store_notify),
};

static int uh_register_attr(struct platform_device *pdev)
{
	int i, error = 0;

	for (i = 0; i < ARRAY_SIZE(uh_sysfs_attrs); i++) {
		error = device_create_file(&pdev->dev, &uh_sysfs_attrs[i]);

		if (error)
			break;
	}

	if (error) {
		while (--i >= 0)
			device_remove_file(&pdev->dev, &uh_sysfs_attrs[i]);
	}

	return 0;
}

static void uh_unregister_attr(struct platform_device *pdev)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(uh_sysfs_attrs); i++)
		device_remove_file(&pdev->dev, &uh_sysfs_attrs[i]);
}

static int uh_setup(struct platform_device *pdev, int gpio_irq, int gpio_pin)
{
	struct uh_data *uh;

	unsigned long status = 0;

        int rv;

	g_puh_data = (struct uh_data *)kzalloc(sizeof(struct uh_data), GFP_KERNEL);
	if (!g_puh_data) {
		printk(KERN_ERR JZ_VH_PFX": Failed to allocate memory.\n");
		return -ENOMEM;
	}

	uh = g_puh_data;

	uh->pdev = pdev;
	uh->gpio_irq = gpio_irq;
	uh->gpio_pin = gpio_pin;

	set_bit(1, &status);

	init_waitqueue_head(&uh->kthread_wq);
	init_waitqueue_head(&uh->timer_wq);
	init_waitqueue_head(&uh->finish_wq);

	BLOCKING_INIT_NOTIFIER_HEAD(&uh->notifier_head);

	setup_timer(&uh->keep_alive_timer, uh_keep_alive_timer_func, (unsigned long)uh);
	setup_timer(&uh->stable_timer, uh_stable_timer_func, (unsigned long)uh);

	uh->kthread = kthread_run(uh_thread, uh, "kuhd");
	if (IS_ERR(uh->kthread)) {
		printk(KERN_ERR JZ_VH_PFX": Failed to create UDC hotplug monitor thread.\n");
		rv = PTR_ERR(uh->kthread);
		goto err;
	}

	set_bit(2, &status);

	uh_init_gpio(uh);

	rv = uh_register_attr(pdev);
	if (rv) {
		printk(KERN_ERR JZ_VH_PFX": Failed to register UDC sysfs interface.\n");
		goto err;
	}

	set_bit(3, &status);

        rv = request_irq(uh->gpio_irq, uh_irq, 0, JZ_VH_PFX, uh);
        if (rv) {
                printk(KERN_ERR JZ_VH_PFX": Could not get udc hotplug irq %d\n", uh->gpio_irq);
		goto err;
        }

	uh_enable();

	uh_set_counter(DEFAULT_KEEP_ALIVE_TIMER_INTERVAL, DEFAULT_KEEP_ALIVE_COUNTER_LIMIT);

	uh_start_work(uh, DO_DETECT);

	printk(KERN_ERR JZ_VH_PFX": Registered.\n");

	return 0;

err:
	if (test_bit(3, &status)) {
		uh_unregister_attr(pdev);
	}

	if (test_bit(2, &status)) {
		uh->thread_state = UH_THREAD_STATE_START;
		kthread_stop(uh->kthread);
	}

	if (test_bit(1, &status)) {
		kfree(g_puh_data);
	}

	return rv;
}

static int musb_gadget_hotplug_setup(struct musb *musb)
{
	struct platform_device *pdev = (struct platform_device *)
		container_of(musb->controller, struct platform_device, dev);
	int rv;

	rv = uh_setup(pdev, OTG_HOTPLUG_IRQ, OTG_HOTPLUG_PIN);
	if (rv)
		printk("uh_setup failed.\n");

	return rv;
}

static void uh_cleanup(struct platform_device *pdev)
{
	struct uh_data *uh = g_puh_data;

	uh_disable();

        free_irq(uh->gpio_irq, uh);

	/* Let our thread to exit. */
	uh->thread_state = UH_THREAD_STATE_START;
        kthread_stop(uh->kthread);

	uh_unregister_attr(pdev);
        kfree(uh);

        return;
}

