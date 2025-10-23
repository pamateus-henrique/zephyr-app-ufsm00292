#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "ethernet_example.h"

int main(void)
{
    printk("=== KSZ8851SNL Ethernet Driver Test (SAMD21) ===\n");
    printk("Initializing Ethernet...\n");
    
    ethernet_example_init();
    
    printk("Ethernet initialization complete\n");
    printk("Running...\n");
    
    while (1) {
        k_sleep(K_SECONDS(5));
        printk("Still alive...\n");
    }
    
    return 0;
}