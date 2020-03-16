/*
 * devMouse.h
 *
 *  Created on: 2020. 3. 16.
 *      Author: parkjunho
 */

#ifndef DEVMOUSE_C_
#define DEVMOUSE_C_

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

enum{
	VERBOSE = true,
	MAX_WIDTH = 1280,
	MAX_HEIGHT = 1024,
	VALUE_PRESSED = 1,
	VALUE_RELEASED = 0,
	BITS_PER_BYTE = 8,
	CURSOR_COLOR = 0x00FF00,
};

struct cursor_t{
	int x, y;
};

struct mouse_t{
	struct cursor_t current;
	struct cursor_t pressed, released;
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
	[SYN_MAX]       = "SYN_MAX",
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


void logging(enum loglevel_t loglevel, char *format, ...)
{
    va_list arg;
    static const char *loglevel2str[] = {
        [DEBUG] = "DEBUG",
        [WARN]  = "WARN",
        [ERROR] = "ERROR",
        [FATAL] = "FATAL",
    };

    /* debug message is available on verbose mode */
    if ((loglevel == DEBUG) && (VERBOSE == false))
        return;

    fprintf(stderr, ">>%s<<\t", loglevel2str[loglevel]);

    va_start(arg, format);
    vfprintf(stderr, format, arg);
    va_end(arg);
}

/* misc functions */
int my_ceil(int val, int div)
{
    if (div == 0)
        return 0;
    else
        return (val + div - 1) / div;
}

/* framebuffer functions */
bool fb_init(struct fb_t *fb)
{
	struct fb_fix_screeninfo finfo;
	struct fb_var_screeninfo vinfo;

	if ((fb->fd = open(fb_dev, O_RDWR)) < 0) {
		perror("open");
		return false;
	}

	if (ioctl(fb->fd, FBIOGET_FSCREENINFO, &finfo)) {
		logging(ERROR, "ioctl: FBIOGET_FSCREENINFO failed\n");
		return false;
	}

	if (ioctl(fb->fd, FBIOGET_VSCREENINFO, &vinfo)) {
		logging(ERROR, "ioctl: FBIOGET_VSCREENINFO failed\n");
		return false;
	}

	fb->width  = vinfo.xres;
	fb->height = vinfo.yres;
	fb->screen_size = finfo.smem_len;
	fb->line_length = finfo.line_length;
	fb->bits_per_pixel  = vinfo.bits_per_pixel;
	fb->bytes_per_pixel = my_ceil(fb->bits_per_pixel, BITS_PER_BYTE);

	if ((fb->fp = (unsigned char *) mmap(0, fb->screen_size,
		PROT_WRITE | PROT_READ, MAP_SHARED, fb->fd, 0)) == MAP_FAILED)
		return false;

	return true;
}

void fb_die(struct fb_t *fb)
{
	munmap(fb->fp, fb->screen_size);
	close(fb->fd);
}

void fb_draw(struct fb_t *fb, struct cursor_t *cursor, uint32_t color)
{
	memcpy(fb->fp + cursor->y * fb->line_length + cursor->x * fb->bytes_per_pixel, &color, fb->bytes_per_pixel);
}

/* mouse functions */
void init_mouse(struct mouse_t *mouse)
{
	mouse->current.x  = mouse->current.y  = 0;
	mouse->pressed.x  = mouse->pressed.y  = 0;
	mouse->released.x = mouse->released.y = 0;
	//mouse->button_pressed  = false;
	//mouse->button_released = false;
}

void print_event(struct input_event *ie)
{
	switch (ie->type) {
	case EV_SYN:
		fprintf(stderr, "time:%ld.%06ld\ttype:%s\tcode:%s\tvalue:%d\n",
			ie->time.tv_sec, ie->time.tv_usec, ev_type[ie->type],
			ev_code_syn[ie->code], ie->value);
		break;
	case EV_REL:
		fprintf(stderr, "time:%ld.%06ld\ttype:%s\tcode:%s\tvalue:%d\n",
			ie->time.tv_sec, ie->time.tv_usec, ev_type[ie->type],
			ev_code_rel[ie->code], ie->value);
		break;
	case EV_KEY:
		fprintf(stderr, "time:%ld.%06ld\ttype:%s\tcode:%s\tvalue:%d\n",
			ie->time.tv_sec, ie->time.tv_usec, ev_type[ie->type],
			ev_code_key[ie->code], ie->value);
		break;
	default:
		break;
	}
}

void print_mouse_state(struct mouse_t *mouse)
{
	fprintf(stderr, "\033[1;1H\033[2Kcurrent(%d, %d) pressed(%d, %d) released(%d, %d)",
		mouse->current.x, mouse->current.y,
		mouse->pressed.x, mouse->pressed.y,
		mouse->released.x, mouse->released.y);
}

void cursor(struct input_event *ie, struct mouse_t *mouse)
{
	if (ie->code == REL_X)
		mouse->current.x += ie->value;

	if (mouse->current.x < 0)
		mouse->current.x = 0;
	else if (mouse->current.x >= MAX_WIDTH)
		mouse->current.x = MAX_WIDTH - 1;

	if (ie->code == REL_Y)
		mouse->current.y += ie->value;

	if (mouse->current.y < 0)
		mouse->current.y = 0;
	else if (mouse->current.y >= MAX_HEIGHT)
		mouse->current.y = MAX_HEIGHT - 1;
}

void button(struct input_event *ie, struct mouse_t *mouse)
{
	if (ie->code != BTN_LEFT)
		return;

	if (ie->value == VALUE_PRESSED)
		mouse->pressed = mouse->current;

	if (ie->value == VALUE_RELEASED)
		mouse->released = mouse->current;
}

/*
void sync(struct input_event *ie, struct mouse_t *mouse)
{
	if (ie->code != SYN_REPORT)
		return;
}
*/

void (*event_handler[EV_CNT])(struct input_event *ie, struct mouse_t *mouse) = {
	//[EV_SYN] = sync,
	[EV_REL] = cursor,
	[EV_KEY] = button,
	[EV_MAX] = NULL,
};

#endif /* DEVMOUSE_C_ */
