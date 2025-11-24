# KSZ8851SNL Testing Notes

## TX Validation

Successful DHCP Discover packet transmission confirmed via Wireshark capture.

### Test Setup
- Target: SAMD21 Xplained Pro
- Network: Direct Ethernet connection
- Tool: Wireshark packet capture

---

# Testing Instructions

Please monitor the serial output after flashing. You should now see:

```
*** Booting Zephyr OS build v4.1.0 ***
[00:00:00.000,000] <inf> eth_ksz8851: ==== KSZ8851SNL driver init called ====
[00:00:00.000,000] <inf> eth_ksz8851: Device: ksz8851@0
[00:00:00.000,000] <inf> eth_ksz8851: Reset sequence completed
[00:00:00.000,000] <inf> eth_ksz8851: Interrupt configured
[00:00:00.000,000] <inf> eth_ksz8851: KSZ8851 CIDER: 0x8872
[00:00:00.000,000] <inf> eth_ksz8851: MAC address set to 00:10:20:30:40:50
[00:00:00.000,000] <inf> eth_ksz8851: KSZ8851SNL initialized successfully
[00:00:00.000,000] <inf> eth_ksz8851: KSZ8851SNL driver initialized
[00:00:00.000,000] <inf> eth_ksz8851: Interface initialized
[00:00:00.500,000] <inf> app: === KSZ8851SNL Ethernet Driver Test ===
[00:00:00.500,000] <inf> app: Network interface found
...
```

If you still see "No network interface found!", please paste the complete serial output here.

The device has been flashed with the new firmware that includes debug logging.

