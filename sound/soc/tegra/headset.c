/*
 *  Headset device detection driver.
 *
 * Copyright (C) 2011 ASUSTek Corporation.
 *
 * Authors:
 *  Jason Cheng <jason4_cheng@asus.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/timer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <asm/string.h>
#include <mach/board-ventana-misc.h>
#include <sound/soc.h>
#include "../codecs/codec_param.h"

MODULE_DESCRIPTION("Headset detection driver");
MODULE_LICENSE("GPL");

/*----------------------------------------------------------------------------
** FUNCTION DECLARATION
**----------------------------------------------------------------------------*/
static int   __init     	headset_init(void);
static void __exit    headset_exit(void);
static irqreturn_t   	detect_irq_handler(int irq, void *dev_id);
static void 		detection_work(struct work_struct *work);
static int               	jack_config_gpio(void);
static irqreturn_t   	lineout_irq_handler(int irq, void *dev_id);
static void 		lineout_work_queue(struct work_struct *work);
static int               	lineout_config_gpio(void);
static void 		detection_work(struct work_struct *work);
static int               	btn_config_gpio(void);
int 			hs_micbias_power(int on);
static irqreturn_t	button_irq_handler(int irq, void *dev_id);
static void	btn_work_queue(struct work_struct *work);
static int	headset_create_input_dev(void);
/*----------------------------------------------------------------------------
** GLOBAL VARIABLES
**----------------------------------------------------------------------------*/
#define JACK_GPIO		178	/* TEGRA_GPIO_PW2 */
#define LINEOUT_GPIO		85		/* TEGRA_GPIO_PK5 */
#define HOOK_GPIO		185		/* TEGRA_GPIO_PX1 */
#define ON	1
#define OFF	0

enum{
	NO_DEVICE	= 0,
	HEADSET	= 2,
};

struct headset_data {
	struct switch_dev sdev;
	struct input_dev *input;
	unsigned int irq;
	struct hrtimer timer;
	ktime_t debouncing_time;
	atomic_t btn_state;
	int ignore_btn;
	unsigned int irq_btn;
	struct hrtimer btn_timer;
	ktime_t btn_debouncing_time;
};

static struct headset_data *hs_data;
bool jack_alive;
EXPORT_SYMBOL(jack_alive);
bool lineout_alive;
EXPORT_SYMBOL(lineout_alive);

int ifininit = 0;
int hook_count = 0;
unsigned long D_delay = 110;
int BTN_F = 0;
int PRJID = 0;
struct timer_list BTN_TIMER;
void BTN_timer_function(unsigned long i);
int BTN_start_timer(unsigned long delay);

static struct workqueue_struct *g_detection_work_queue;
static DECLARE_WORK(g_detection_work, detection_work);

struct work_struct headset_work;
struct work_struct lineout_work;
struct work_struct work_btn;
extern struct snd_soc_codec *global_codec;
extern bool need_spk;
extern int PRJ_ID;
extern struct wm8903_parameters audio_params[];

int BTN_start_timer(unsigned long delay)
{
	int retval = 0;

	retval = del_timer(&BTN_TIMER);
	init_timer(&BTN_TIMER);
	BTN_TIMER.expires = jiffies+(u32)delay;
	BTN_TIMER.data = 0;
	BTN_TIMER.function = BTN_timer_function;
	add_timer(&BTN_TIMER);

	return retval;
}
void BTN_timer_function(unsigned long i)
{
	BTN_F = 0;
}

static ssize_t headset_name_show(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs_data->sdev)){
	case NO_DEVICE:{
		return sprintf(buf, "%s\n", "No Device");
		}
	case HEADSET:{
		return sprintf(buf, "%s\n", "Headset");
		}
	}
	return -EINVAL;
}

static ssize_t headset_state_show(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs_data->sdev)){
	case NO_DEVICE:
		return sprintf(buf, "%d\n", 0);
	case HEADSET:
		return sprintf(buf, "%d\n", 2);
	}
	return -EINVAL;
}

static void insert_headset(void)
{
	hs_micbias_power(1);
	msleep(100);

	snd_soc_write(global_codec, 0x0e, 0x3); /* Enable HP output*/
	switch_set_state(&hs_data->sdev, HEADSET);
	hs_data->debouncing_time = ktime_set(0, 20000000);  /* 20 ms */
	jack_alive = true;
}

static void remove_headset(void)
{
	hs_micbias_power(0);
	snd_soc_write(global_codec, 0x0e, 0x0);	/* Disable HP output*/
	switch_set_state(&hs_data->sdev, NO_DEVICE);
	hs_data->debouncing_time = ktime_set(0, 100000000);  /* 100 ms */
	jack_alive = false;
}

int check_hs_type(void)
{
	if(jack_alive){
		hs_micbias_power(0);
		hs_micbias_power(1);
	}
	msleep(100);
	/* For No Mic dongle */
	if(!gpio_get_value(JACK_GPIO)){
		if (gpio_get_value(HOOK_GPIO)){
			printk("HEADSET: No mic headset\n");
			return 0;
		}else{
			printk("HEADSET: With mic headset\n");
			return 1;
		}
	}else{
		printk("HEADSET: No headset plug-in\n");
		return 0;
	}
}
EXPORT_SYMBOL(check_hs_type);

static void btn_work_queue(struct work_struct *work)
{
	if(switch_get_state(&hs_data->sdev) == NO_DEVICE || gpio_get_value(JACK_GPIO)){
		msleep(200);
	}else{
		msleep(10);  /* Button debucing time */

		if(!gpio_get_value(JACK_GPIO)){			/* Headset Plug-in */
			if(switch_get_state(&hs_data->sdev) != NO_DEVICE){
			if(gpio_get_value(HOOK_GPIO)){
				if(BTN_F != 0)
					goto out;

			input_report_key(hs_data->input, 233, 1);/* Key_HeadsetHook Down */

			if(!gpio_get_value(HOOK_GPIO)){
				input_report_key(hs_data->input, 233, 0);/* Key_HeadsetHook Up */
			}
			}else{
				input_report_key(hs_data->input, 233, 0);/* Key_HeadsetHook Up */
			}
				hook_count = 0;
		}
		}
       }

out:
	hook_count = 0;
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
	int state = 0;

	if(!ifininit){
		schedule_work(&work_btn) ;
		hook_count++;
	}
	return IRQ_HANDLED;
}

static void detection_work(struct work_struct *work)
{
	unsigned long irq_flags;
	int cable_in1;

	hs_micbias_power(0);
	del_timer(&BTN_TIMER);
	BTN_F = 1;

	/* Disable headset interrupt while detecting.*/
	local_irq_save(irq_flags);
	disable_irq(hs_data->irq);
	local_irq_restore(irq_flags);

	/* Delay 500ms for pin stable. */
	msleep(500);

	/* Restore IRQs */
	local_irq_save(irq_flags);
	enable_irq(hs_data->irq);
	local_irq_restore(irq_flags);

	if (gpio_get_value(JACK_GPIO) != 0) {
		/* Headset not plugged in */
		if (switch_get_state(&hs_data->sdev) == HEADSET)
			remove_headset();
		BTN_start_timer(310);
		return;
	}

	cable_in1 = gpio_get_value(JACK_GPIO);

	if (cable_in1 == 0) {
	       BTN_start_timer(310);
		if(switch_get_state(&hs_data->sdev) == NO_DEVICE){
		       hook_count=0;
			insert_headset();
		}else{
			hs_micbias_power(1);
		}
	} else{
		BTN_start_timer(310);
		printk("HEADSET: Jack-in GPIO is low, but not a headset \n");
	}
}

static enum hrtimer_restart detect_event_timer_func(struct hrtimer *data)
{
	queue_work(g_detection_work_queue, &g_detection_work);
	return HRTIMER_NORESTART;
}

/**********************************************************
**  Function: Jack detection-in gpio configuration function
**  Parameter: none
**  Return value: if sucess, then returns 0
**
************************************************************/
static int jack_config_gpio()
{
	int ret;
	
	printk("HEADSET: Config Jack-in detection gpio\n");

	tegra_gpio_enable(JACK_GPIO);
	ret = gpio_request(JACK_GPIO, "h2w_detect");
	ret = gpio_direction_input(JACK_GPIO);

	hs_data->irq = gpio_to_irq(JACK_GPIO);
	ret = request_irq(hs_data->irq, detect_irq_handler,
			  IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "h2w_detect", NULL);

	ret = set_irq_wake(hs_data->irq, 1);

	if (gpio_get_value(JACK_GPIO) == 0){
		jack_alive = true;
		insert_headset();
	}else {
		jack_alive = false;
		remove_headset();
	}

	return 0;
}

/**********************************************************
**  Function: Headset Hook Key Detection interrupt handler
**  Parameter: irq  
**  Return value: IRQ_HANDLED
**  High: Hook button pressed
************************************************************/
static int btn_config_gpio()
{
	int ret;

	printk("HEADSET: Config Headset Button detection gpio\n");

	tegra_gpio_enable(HOOK_GPIO);
	ret = gpio_request(HOOK_GPIO, "btn_INT");
	ret = gpio_direction_input(HOOK_GPIO);

	if(PRJID == 102){
		hs_data->irq_btn = gpio_to_irq(HOOK_GPIO);

		ret = request_irq(hs_data->irq_btn, &button_irq_handler, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "btn_INT", 0);
		if(ret)
			printk("HEADSET: btn_config_gpio: request_irq failed for gpio %d\n", HOOK_GPIO);
	}
}

static void lineout_work_queue(struct work_struct *work)
{
	msleep(300);

	if (gpio_get_value(LINEOUT_GPIO) == 0){
		printk("LINEOUT: LineOut inserted\n");
		lineout_alive = true;
		snd_soc_write(global_codec, 0x10, 0x0000); /* MIXSPK Disable*/
		snd_soc_write(global_codec, 0x11, 0x0000); /* SPK Disable*/
		snd_soc_write(global_codec, 0x76, 0x0000); /* Mute the speaker */

	}else if(gpio_get_value(LINEOUT_GPIO) && need_spk){
		printk("LINEOUT: LineOut removed\n");
		lineout_alive = false;
		snd_soc_write(global_codec, 0x10, 0x0003); /* MIXSPK Enable*/
		if(PRJ_ID == 101){
		snd_soc_write(global_codec, 0x3E, audio_params[EP101].analog_speaker_volume | 0x80); /* SPKL Volume: 4dB*/
		snd_soc_write(global_codec, 0x3F, audio_params[EP101].analog_speaker_volume | 0x80); /* SPKR Volume: 4dB*/
		}else if(PRJ_ID == 102){
		snd_soc_write(global_codec, 0x3E, audio_params[EP102].analog_speaker_volume | 0x80); /* SPKL Volume: 0dB*/
		snd_soc_write(global_codec, 0x3F,audio_params[EP102].analog_speaker_volume | 0x80); /* SPKR Volume: 0dB*/
		}
		snd_soc_write(global_codec, 0x11, 0x0003); /* SPK Enable*/
		snd_soc_write(global_codec, 0x76, 0x0033); /* GPIO3 configure: EN_SPK*/
	}

}

/**********************************************************
**  Function: LineOut Detection configuration function
**  Parameter: none
**  Return value: IRQ_HANDLED
**
************************************************************/
static int lineout_config_gpio()
{
	int ret;

	printk("HEADSET: Config LineOut detection gpio\n");

	tegra_gpio_enable(LINEOUT_GPIO);
	ret = gpio_request(LINEOUT_GPIO, "lineout_int");
	ret = gpio_direction_input(LINEOUT_GPIO);
	ret = request_irq(gpio_to_irq(LINEOUT_GPIO), &lineout_irq_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "lineout_int", 0);

	if (gpio_get_value(LINEOUT_GPIO) == 0)
		lineout_alive = true;
	else
		lineout_alive = false;

	return 0;
}

/**********************************************************
**  Function: LineOut detection interrupt handler
**  Parameter: dedicated irq
**  Return value: if sucess, then returns IRQ_HANDLED
**
************************************************************/
static irqreturn_t lineout_irq_handler(int irq, void *dev_id)
{
	schedule_work(&lineout_work);
	return IRQ_HANDLED;
}

/**********************************************************
**  Function: Headset jack-in detection interrupt handler
**  Parameter: dedicated irq
**  Return value: if sucess, then returns IRQ_HANDLED
**
************************************************************/
static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	do {
		value1 = gpio_get_value(JACK_GPIO);
		set_irq_type(hs_data->irq, value1 ?
				IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
		value2 = gpio_get_value(JACK_GPIO);
	} while (value1 != value2 && retry_limit-- > 0);

	if ((switch_get_state(&hs_data->sdev) == NO_DEVICE) ^ value2)
		hrtimer_start(&hs_data->timer, hs_data->debouncing_time, HRTIMER_MODE_REL);

	return IRQ_HANDLED;
}

int hs_micbias_power(int on)
{
	static int nLastVregStatus = -1;
	int CtrlReg = 0;

	if(on && nLastVregStatus!=ON){
		printk("HEADSET: Turn on micbias power\n");
		nLastVregStatus = ON;

		/* Mic Bias enable */
		CtrlReg = (0x1<<0) | (0x1<<1);
		snd_soc_write(global_codec, 0x06, CtrlReg);
		
		/* Enable analog inputs */
		CtrlReg = (0x1<<1) | (0x1<<0);
		snd_soc_write(global_codec, 0x0C, CtrlReg);
		
	}else if(!on && nLastVregStatus!=OFF){
		printk("HEADSET: Turn off micbias power\n");
		nLastVregStatus = OFF;
		
		/* Mic Bias disable */
		CtrlReg = (0x0<<0) | (0x0<<1);
		snd_soc_write(global_codec, 0x06, CtrlReg);
		
		/* Disable analog inputs */
		CtrlReg = (0x0<<1) | (0x0<<0);
		snd_soc_write(global_codec, 0x0C, CtrlReg);	

	}
	return 0;
}
EXPORT_SYMBOL(hs_micbias_power);

static int headset_create_input_dev(void)
{
	int err = 0;

	/* Device related initialization */
	hs_data->input = input_allocate_device();
	if (!hs_data->input) {
		err = -ENOMEM;
		goto out;
	}

	printk("HEADSET: device allocated\n");

	hs_data->input->name = "Wired Headset";
	hs_data->input->phys = "/dev/input/headset";

	set_bit(EV_SYN, hs_data->input->evbit);
	set_bit(EV_KEY, hs_data->input->evbit);
	set_bit(233, hs_data->input->keybit);

	/* Register the Device */
	err = input_register_device(hs_data->input);
	if (err){
		printk("HEADSET: Unable to register %s input device\n", hs_data->input->name);
		goto free_out;
        }

	return 0;

free_out:
	input_free_device(hs_data->input);
out:
	return err;
}

/**********************************************************
**  Function: Headset driver init function
**  Parameter: none  
**  Return value: none
**                     
************************************************************/
static int __init headset_init(void)
{
	int ret;

	printk("HEADSET: Headset detection init\n");

	hs_data = kzalloc(sizeof(struct headset_data), GFP_KERNEL);
	if (!hs_data)
		return -ENOMEM;

	hs_data->debouncing_time = ktime_set(0, 100000000);  /* 100 ms */
	hs_data->btn_debouncing_time = ktime_set(0, 10000000); /* 10 ms */
	hs_data->sdev.name = "h2w";
	hs_data->sdev.print_name = headset_name_show;
	hs_data->sdev.print_state = headset_state_show;

	ret = switch_dev_register(&hs_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	g_detection_work_queue = create_workqueue("detection");

	hrtimer_init(&hs_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hs_data->timer.function = detect_event_timer_func;
	init_timer(&BTN_TIMER);

	PRJID = ASUSGetProjectID();

	if (PRJID == 101){
		INIT_WORK(&lineout_work, lineout_work_queue);
		lineout_config_gpio();	
	}else if(PRJID == 102){
		INIT_WORK(&work_btn, btn_work_queue);

		ret = headset_create_input_dev();
		if (ret) {
			printk(KERN_WARNING "HEADSET: Error creating input device: %d\n", ret);
		}
		printk("HEADSET: headset_create_input_dev(client), err: %d(0:success)\n",ret);
	}

	printk("HEADSET: Headset detection mode\n");
	jack_config_gpio();/*Config jack detection GPIO*/
	btn_config_gpio();/*Config hook detection GPIO*/

	return 0;

err_switch_dev_register:
	printk(KERN_ERR "Headset: Failed to register driver\n");

	return ret;
}

/**********************************************************
**  Function: Headset driver exit function
**  Parameter: none
**  Return value: none
**
************************************************************/
static void __exit headset_exit(void)
{
	printk("HEADSET: Headset exit\n");
	if (switch_get_state(&hs_data->sdev))
		remove_headset();
	gpio_free(JACK_GPIO);
	gpio_free(HOOK_GPIO);

	if (ASUSGetProjectID() == 101){
		gpio_free(LINEOUT_GPIO);
	}

	free_irq(hs_data->irq, 0);
	if(PRJID == 102)
		free_irq(hs_data->irq_btn, 0);
	destroy_workqueue(g_detection_work_queue);
	switch_dev_unregister(&hs_data->sdev);
}

module_init(headset_init);
module_exit(headset_exit);
