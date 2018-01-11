
#ifndef __LINUX_ATMEL_MXT_PLUG_WDG
#define __LINUX_ATMEL_MXT_PLUG_WDG

#include <linux/types.h>

ssize_t plugin_wdg_show(struct plugin_wdg *p, char *buf, size_t count);
int plugin_wdg_store(struct plugin_wdg *p, const char *buf, size_t count);
int plugin_interface_wdg_init(struct plugin_wdg *p);

#endif


