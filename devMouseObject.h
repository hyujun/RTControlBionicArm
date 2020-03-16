/*
 * devMouseObject.h
 *
 *  Created on: 2020. 3. 16.
 *      Author: parkjunho
 */

#ifndef DEVMOUSEOBJECT_H_
#define DEVMOUSEOBJECT_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <linux/input.h>
#include <fcntl.h>

#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include <linux/fb.h>
#include <sys/mman.h>

class devMouseObject {

public:
	devMouseObject();
	virtual ~devMouseObject();

	void logging(enum loglevel_t loglevel, char *format, ...);
	int my_ceil(int val, int div);
	bool fb_init(struct fb_t *fb);
	void fb_die(struct fb_t *fb);
	void fb_draw(struct fb_t *fb, struct cursor_t *cursor, uint32_t color);
	void init_mouse(struct mouse__t *mouse);
	void print_event(struct input_event *ie);
	void print_mouse_state(struct mouse_t *mouse);
	void cursor(struct input_event *ie, struct mouse_t *mouse);
	void button(struct input_event *ie, struct mouse_t *mouse);
	void (*event_handler[EV_CNT])(struct input_event *ie, struct mouse_t *mouse) = {
		//[EV_SYN] = sync,
		[EV_REL] = cursor,
		[EV_KEY] = button,
		[EV_MAX] = NULL,
	};

	enum {
		VERBOSE    = true,
		MAX_WIDTH  = 1280,
		MAX_HEIGHT = 1024,
		VALUE_PRESSED  = 1,
		VALUE_RELEASED = 0,
		BITS_PER_BYTE  = 8,
		CURSOR_COLOR   = 0x00FF00,
	};

	struct cursor_t {
		int x, y;
	};

	struct mouse_t {
		struct cursor_t current;
		struct cursor_t pressed, released;
		//bool button_pressed;
		//bool button_released;
	};

	struct fb_t {
	    int fd;
	    unsigned char *fp;
	    int width, height;   /* display resolution */
	    long screen_size;    /* screen data size (byte) */
	    int line_length;     /* line length (byte) */
	    int bytes_per_pixel;
	    int bits_per_pixel;
	};

	//const char *mouse_dev = "/dev/input/event8";
	//const char *mouse_dev = "/dev/input/event9";
	const char *mouse_dev = "/dev/input/by-path/pci-0000:00:1a.0-usb-0:1.2:1.1-event-mouse";
	const char *fb_dev    = "/dev/fb0";

	const char *ev_type[EV_CNT] = {
		[EV_SYN]       = "EV_SYN",
		[EV_KEY]       = "EV_KEY",
		[EV_REL]       = "EV_REL",
		[EV_ABS]       = "EV_ABS",
		[EV_MSC]       = "EV_MSC",
		[EV_SW]        = "EV_SW",
		[EV_LED]       = "EV_LED",
		[EV_SND]       = "EV_SND",
		[EV_REP]       = "EV_REP",
		[EV_FF]        = "EV_FF",
		[EV_PWR]       = "EV_PWR",
		[EV_FF_STATUS] = "EV_FF_STATUS",
		[EV_MAX]       = "EV_MAX",
	};

	const char *ev_code_syn[SYN_CNT] = {
		[SYN_REPORT]    = "SYN_REPORT",
		[SYN_CONFIG]    = "SYN_CONFIG",
		[SYN_MT_REPORT] = "SYN_MT_REPORT",
		[SYN_DROPPED]   = "SYN_DROPPED",
	};

	const char *ev_code_rel[REL_CNT] = {
		[REL_X]      = "REL_X",
		[REL_Y]      = "REL_Y",
		[REL_Z]      = "REL_Z",
		[REL_RX]     = "REL_RX",
		[REL_RY]     = "REL_RY",
		[REL_RZ]     = "REL_RZ",
		[REL_HWHEEL] = "REL_WHEEL",
		[REL_DIAL]   = "REL_DIAL",
		[REL_WHEEL]  = "REL_WHEEL",
		[REL_MISC]   = "REL_MISC",
		[REL_MAX]    = "REL_MAX",
	};

	const char *ev_code_key[KEY_CNT] = {
		[BTN_LEFT]    = "BTN_LEFT",
		[BTN_RIGHT]   = "BTN_RIGHT",
		[BTN_MIDDLE]  = "BTN_MIDDLE",
		[BTN_SIDE]    = "BTN_SIDE",
		[BTN_EXTRA]   = "BTN_EXTRA",
		[BTN_FORWARD] = "BTN_FORWARD",
		[BTN_BACK]    = "BTN_BACK",
		[BTN_TASK]    = "BTN_TASK",
		[KEY_MAX]     = "KEY_MAX",
	};

	/* logging functions */
	enum loglevel_t {
	    DEBUG = 0,
	    WARN,
	    ERROR,
	    FATAL,
	};
};

#endif /* DEVMOUSEOBJECT_H_ */
