/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include<zephyr/logging/log.h>

#include "demo.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);


int main(void)
{
	bool rv = demoIni();
    if (!rv)
	{
        LOG_ERR("failed to initialize");
    }
	else
	{
        while (1)
		{
            demoCycle();
        }
    }

	return 0;
}
