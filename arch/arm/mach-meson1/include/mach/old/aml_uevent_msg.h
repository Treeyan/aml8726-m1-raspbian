/*
 *
 *
 * Author: Peter Lin <peter.lin@amlogic.com>
 *
 *
 * Copyright (C) 2011 Amlogic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __AML_UEVENT_MSG_H
#define __AML_UEVENT_MSG_H

/* Standard Linux headers */
#include <linux/types.h>
#include <linux/device.h>

#ifdef CONFIG_AML_UEVENT_MSG
int aml_send_msg(char *error_name, int error_flag);
#else
#define aml_send_msg(a,b)	do{}while(0);
#endif


#endif //__AML_UEVENT_MSG_H

