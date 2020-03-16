/*
 * devMouseObject.cpp
 *
 *  Created on: 2020. 3. 16.
 *      Author: parkjunho
 */

#include "devMouseObject.h"

devMouseObject::devMouseObject() {
	// TODO Auto-generated constructor stub

}

devMouseObject::~devMouseObject() {
	// TODO Auto-generated destructor stub
}


void devMouseObject::logging(enum loglevel_t loglevel, char *format, ...)
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
