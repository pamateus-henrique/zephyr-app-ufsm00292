#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
    printk("=== Console Test ===\n");
    printk("If you see this, console works!\n");
    
    while (1) {
        printk("Alive: %lld\n", k_uptime_get());
        k_sleep(K_SECONDS(1));
    }
    
    return 0;
}