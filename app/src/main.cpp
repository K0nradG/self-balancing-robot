#include <zephyr/sys/printk.h>

#ifdef CONFIG_BATTERY_LEVEL_DRV
#include "battery_level.h"
#endif

int main(){
    printk("dummy app");
}
