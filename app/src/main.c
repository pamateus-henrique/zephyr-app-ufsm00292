#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/ethernet.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

static struct net_mgmt_event_callback mgmt_cb;

static void event_handler(struct net_mgmt_event_callback *cb,
			  uint32_t mgmt_event, struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (mgmt_event == NET_EVENT_IPV4_DHCP_BOUND) {
		LOG_INF("DHCP bound - IPv4 address assigned");
	} else if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
		LOG_INF("IPv4 address added to interface");
	} else if (mgmt_event == NET_EVENT_IPV4_DHCP_START) {
		LOG_INF("DHCP client started");
	}
}

int main(void)
{
	struct net_if *iface;
	struct net_linkaddr *link_addr;
	const struct device *eth_dev;
	const struct device *spi_dev;

	LOG_INF("=== KSZ8851SNL Ethernet Driver Test ===");

	/* Check if SPI device is ready */
	spi_dev = DEVICE_DT_GET(DT_NODELABEL(sercom0));
	if (device_is_ready(spi_dev)) {
		LOG_INF("SPI device (sercom0) is ready");
	} else {
		LOG_ERR("SPI device (sercom0) NOT ready!");
	}

	/* Try to get the Ethernet device directly */
	eth_dev = DEVICE_DT_GET(DT_NODELABEL(ksz8851));
	if (eth_dev) {
		LOG_INF("Found ethernet device: %s", eth_dev->name);
		LOG_INF("Device ready: %s", device_is_ready(eth_dev) ? "YES" : "NO");
	} else {
		LOG_ERR("Could not find ksz8851 device!");
	}

	/* Wait a bit for the driver to initialize */
	k_sleep(K_MSEC(500));

	/* Get the default network interface */
	iface = net_if_get_default();
	if (!iface) {
		LOG_ERR("No network interface found!");
		return -1;
	}

	LOG_INF("Network interface found");
	
	/* Print MAC address */
	link_addr = net_if_get_link_addr(iface);
	if (link_addr && link_addr->len == 6) {
		LOG_INF("MAC: %02x:%02x:%02x:%02x:%02x:%02x",
			link_addr->addr[0], link_addr->addr[1], link_addr->addr[2],
			link_addr->addr[3], link_addr->addr[4], link_addr->addr[5]);
	}
	
	/* Register network management callback */
	net_mgmt_init_event_callback(&mgmt_cb, event_handler,
				     NET_EVENT_IPV4_ADDR_ADD |
				     NET_EVENT_IPV4_DHCP_START |
				     NET_EVENT_IPV4_DHCP_BOUND);
	net_mgmt_add_event_callback(&mgmt_cb);
	
	/* Start DHCP client */
	LOG_INF("Starting DHCP client...");
	net_dhcpv4_start(iface);
	
	LOG_INF("Application running - waiting for DHCP...");

	/* Main loop */
	int poll_count = 0;
	while (1) {
		k_sleep(K_SECONDS(5));

		poll_count++;

		/* Every 5 seconds, check status */
		if (poll_count % 6 == 0) {  /* Every 30 seconds */
			if (net_if_is_up(iface)) {
				bool carrier = net_if_flag_is_set(iface, NET_IF_LOWER_UP);
				LOG_INF("Status: Interface UP, Link %s",
					carrier ? "UP" : "DOWN");
			} else {
				LOG_INF("Status: Interface DOWN");
			}
		}
	}
	
	return 0;
}
