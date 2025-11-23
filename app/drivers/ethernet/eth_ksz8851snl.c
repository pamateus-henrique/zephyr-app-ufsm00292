/*
 * KSZ8851SNL Ethernet Driver for Zephyr
 * Full network stack integration with TX/RX support
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <string.h>

LOG_MODULE_REGISTER(eth_ksz8851, LOG_LEVEL_DBG);

#define DT_DRV_COMPAT microchip_ksz8851snl

/* KSZ8851SNL Register Map */
#define KSZ_REG_MARL       0x0010  /* MAC Address Low */
#define KSZ_REG_MARM       0x0012  /* MAC Address Middle */
#define KSZ_REG_MARH       0x0014  /* MAC Address High */
#define KSZ_REG_OBCR       0x0020  /* On-chip Bus Control */
#define KSZ_REG_GRR        0x0026  /* Global Reset */
#define KSZ_REG_TXCR       0x0070  /* TX Control */
#define KSZ_REG_TXSR       0x0072  /* TX Status */
#define KSZ_REG_RXCR1      0x0074  /* RX Control 1 */
#define KSZ_REG_RXCR2      0x0076  /* RX Control 2 */
#define KSZ_REG_TXMIR      0x0078  /* TX Memory Info */
#define KSZ_REG_RXFHSR     0x007C  /* RX Frame Header Status */
#define KSZ_REG_RXFHBCR    0x007E  /* RX Frame Header Byte Count */
#define KSZ_REG_TXQCR      0x0080  /* TX Queue Command */
#define KSZ_REG_RXQCR      0x0082  /* RX Queue Command */
#define KSZ_REG_TXFDPR     0x0084  /* TX Frame Data Pointer */
#define KSZ_REG_RXFDPR     0x0086  /* RX Frame Data Pointer (QMU) */
#define KSZ_REG_IER        0x0090  /* Interrupt Enable */
#define KSZ_REG_ISR        0x0092  /* Interrupt Status */
#define KSZ_REG_RXFCTR     0x009C  /* RX Frame Count & Threshold */
#define KSZ_REG_MAHTR0     0x00A0  /* MAC Address Hash Table 0 */
#define KSZ_REG_P1MBCR     0x00B4  /* PHY MII Basic Control */
#define KSZ_REG_P1SR       0x00B8  /* Port 1 Status */
#define KSZ_REG_CIDER      0x00C0  /* Chip ID and Enable */
#define KSZ_REG_CGCR       0x00C6  /* Chip Global Control */
#define KSZ_REG_IACR       0x00C8  /* Indirect Access Control */
#define KSZ_REG_PMECR      0x00D4  /* Power Management Event Control */
#define KSZ_REG_PHYRR      0x00D8  /* PHY Reset */

/* Register Bit Definitions */
#define PMECR_PME_ENABLE   BIT(0)   /* PME Output Enable */
#define PMECR_PME_POLARITY BIT(1)   /* PME Output Polarity (1=active high, 0=active low) */
#define PMECR_WAKEUP_LINK  BIT(2)   /* Link Change Wake-up Enable */
#define GRR_QMU_RESET      BIT(1)
#define TXCR_TXE           BIT(0)   /* TX Enable */
#define TXCR_TXFCE         BIT(3)   /* TX Flow Control Enable */
#define RXCR1_RXE          BIT(0)   /* RX Enable */
#define RXCR1_RXINVF       BIT(1)   /* RX Inverse Filtering */
#define RXCR1_RXAE         BIT(2)   /* RX All Enable */
#define RXCR1_RXMAFMA      BIT(7)   /* RX MAC Address Filtering */
#define RXCR1_RXFCE        BIT(10)  /* RX Flow Control Enable */
#define RXQCR_RXFCTE       BIT(0)   /* RX Frame Count Threshold Enable */
#define RXQCR_AUTO_DEQUEUE BIT(1)   /* Auto-dequeue RXQ */
#define RXQCR_RRXEF        BIT(12)  /* Release RX Error Frame */
#define TXQCR_METFE        BIT(2)   /* Manual Enqueue TX Frame Enable */
#define TXQCR_TXQMAM       BIT(3)   /* TXQ Memory Available Monitor */
#define IER_LCIE           BIT(0)   /* Link Change Interrupt Enable */
#define IER_TXIE           BIT(1)   /* TX Interrupt Enable */
#define IER_RXIE           BIT(2)   /* RX Interrupt Enable */
#define ISR_LCIS           BIT(0)   /* Link Change */
#define ISR_TXIS           BIT(1)   /* TX */
#define ISR_RXIS           BIT(2)   /* RX */
#define P1SR_LINK_GOOD     BIT(5)   /* Link Status */
#define CIDER_ENABLE       BIT(0)   /* Chip Enable */

/* FIFO sizes */
#define KSZ_TX_FIFO_SIZE   2048
#define KSZ_RX_FIFO_SIZE   2048

/* Frame size limits */
#define KSZ_MAX_FRAME_SIZE 1518

struct ksz8851_runtime {
	struct net_if *iface;
	uint8_t mac_addr[6];
	const struct device *dev;
	struct gpio_callback int_callback;
	struct k_work rx_work;
	struct k_sem tx_sem;
	struct k_timer poll_timer;  /* Polling timer for RX (fallback when interrupts don't work) */
	bool initialized;
	bool link_up;
	/* Statistics */
	uint32_t tx_packets;
	uint32_t rx_packets;
	uint32_t tx_errors;
	uint32_t rx_errors;
};

struct ksz8851_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset;
	struct gpio_dt_spec interrupt;
};

/* SPI Communication Helpers */

static inline void ksz_cmd_read(uint16_t reg, uint8_t *cmd, uint8_t byte_enable)
{
	/* Opcode=00 (read), byte_enable bits 10-13, address bits 2-9
	 * Based on Microchip's reference driver:
	 * cmd = ((read_io << 8) | (byteEnable << 10) | (address << 2))
	 * read_io = 0x00
	 */
	uint16_t command = ((byte_enable << 10) | (reg << 2));
	cmd[0] = (command >> 8) & 0xFF;
	cmd[1] = command & 0xFF;
}

static inline void ksz_cmd_write(uint16_t reg, uint8_t *cmd, uint8_t byte_enable)
{
	/* Opcode=01 (write), byte_enable bits 10-13, address bits 2-9
	 * Based on Microchip's reference driver:
	 * cmd = ((write_io << 8) | (byteEnable << 10) | (address << 2))
	 * write_io = 0x40
	 */
	uint16_t command = ((0x40 << 8) | (byte_enable << 10) | (reg << 2));
	cmd[0] = (command >> 8) & 0xFF;
	cmd[1] = command & 0xFF;
}

static int ksz_read_reg(const struct ksz8851_config *config, uint16_t reg,
			uint8_t *data, uint8_t len)
{
	uint8_t cmd[2];
	uint8_t byte_enable = (1 << len) - 1;
	
	ksz_cmd_read(reg, cmd, byte_enable);
	
	uint8_t tx_buf[2 + 4] = {cmd[0], cmd[1], 0, 0, 0, 0};
	uint8_t rx_buf[2 + 4] = {0};
	
	const struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = 2 + len};
	const struct spi_buf_set tx_spi = {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_buf = {.buf = rx_buf, .len = 2 + len};
	const struct spi_buf_set rx_spi = {.buffers = &rx_spi_buf, .count = 1};
	
	int ret = spi_transceive_dt(&config->spi, &tx_spi, &rx_spi);
	if (ret) {
		return ret;
	}
	
	memcpy(data, &rx_buf[2], len);
	return 0;
}

static int ksz_write_reg(const struct ksz8851_config *config, uint16_t reg,
			 const uint8_t *data, uint8_t len)
{
	uint8_t cmd[2];
	uint8_t byte_enable = (1 << len) - 1;

	ksz_cmd_write(reg, cmd, byte_enable);

	uint8_t tx_buf[2 + 4];
	tx_buf[0] = cmd[0];
	tx_buf[1] = cmd[1];
	memcpy(&tx_buf[2], data, len);

	const struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = 2 + len};
	const struct spi_buf_set tx_spi = {.buffers = &tx_spi_buf, .count = 1};

	return spi_write_dt(&config->spi, &tx_spi);
}

static int ksz_read16(const struct ksz8851_config *config, uint16_t reg, uint16_t *val)
{
	uint8_t buf[2];
	int ret = ksz_read_reg(config, reg, buf, 2);
	if (ret) {
		LOG_ERR("SPI read error at reg 0x%04x: %d", reg, ret);
		return ret;
	}
	*val = buf[0] | ((uint16_t)buf[1] << 8);
	LOG_DBG("Read reg 0x%04x: 0x%04x (bytes: 0x%02x 0x%02x)", reg, *val, buf[0], buf[1]);
	return 0;
}

static int ksz_write16(const struct ksz8851_config *config, uint16_t reg, uint16_t val)
{
	uint8_t buf[2] = {val & 0xFF, (val >> 8) & 0xFF};
	int ret = ksz_write_reg(config, reg, buf, 2);

	/* Debug logging for IER writes */
	if (reg == KSZ_REG_IER) {
		uint8_t cmd[2];
		uint8_t byte_enable = 3;  /* 2 bytes */
		ksz_cmd_write(reg, cmd, byte_enable);
		LOG_INF("ksz_write16: IER reg=0x%04x val=0x%04x data=[0x%02x 0x%02x] cmd=[0x%02x 0x%02x] ret=%d",
			reg, val, buf[0], buf[1], cmd[0], cmd[1], ret);
	}

	return ret;
}

static int ksz_setbits16(const struct ksz8851_config *config, uint16_t reg, uint16_t bits)
{
	uint16_t val;
	int ret = ksz_read16(config, reg, &val);
	if (ret) {
		return ret;
	}
	return ksz_write16(config, reg, val | bits);
}

static int ksz_clrbits16(const struct ksz8851_config *config, uint16_t reg, uint16_t bits)
{
	uint16_t val;
	int ret = ksz_read16(config, reg, &val);
	if (ret) {
		return ret;
	}
	return ksz_write16(config, reg, val & ~bits);
}

/* FIFO Access */

static int ksz_read_fifo(const struct ksz8851_config *config, uint8_t *buf, size_t len)
{
	/* FIFO read uses special opcode */
	uint8_t dummy[8] = {0};
	
	const struct spi_buf tx_bufs[] = {
		{.buf = dummy, .len = 8},  /* 8 dummy bytes before data */
	};
	struct spi_buf rx_bufs[] = {
		{.buf = dummy, .len = 8},
		{.buf = buf, .len = len},
	};
	
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = 1};
	const struct spi_buf_set rx = {.buffers = rx_bufs, .count = 2};
	
	return spi_transceive_dt(&config->spi, &tx, &rx);
}

static int ksz_write_fifo(const struct ksz8851_config *config, const uint8_t *buf, size_t len)
{
	/* FIFO write: opcode + data */
	uint8_t dummy[4] = {0};
	
	const struct spi_buf tx_bufs[] = {
		{.buf = dummy, .len = 4},  /* 4 dummy bytes */
		{.buf = (void *)buf, .len = len},
	};
	
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = 2};
	
	return spi_write_dt(&config->spi, &tx);
}

/* Chip Initialization */

static int ksz_soft_reset(const struct ksz8851_config *config)
{
	int ret;
	
	/* QMU soft reset */
	ret = ksz_write16(config, KSZ_REG_GRR, GRR_QMU_RESET);
	if (ret) {
		return ret;
	}
	
	k_msleep(10);
	
	/* Clear the reset */
	ret = ksz_write16(config, KSZ_REG_GRR, 0);
	if (ret) {
		return ret;
	}
	
	k_msleep(10);
	return 0;
}

static int ksz_set_mac_address(const struct ksz8851_config *config, const uint8_t *mac)
{
	int ret;
	uint16_t val;
	
	/* Write MAC address (low, middle, high) */
	val = mac[0] | ((uint16_t)mac[1] << 8);
	ret = ksz_write16(config, KSZ_REG_MARL, val);
	if (ret) {
		return ret;
	}
	
	val = mac[2] | ((uint16_t)mac[3] << 8);
	ret = ksz_write16(config, KSZ_REG_MARM, val);
	if (ret) {
		return ret;
	}
	
	val = mac[4] | ((uint16_t)mac[5] << 8);
	ret = ksz_write16(config, KSZ_REG_MARH, val);
	if (ret) {
		return ret;
	}
	
	LOG_INF("MAC address set to %02x:%02x:%02x:%02x:%02x:%02x",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	
	return 0;
}

static int ksz_init_chip(const struct device *dev)
{
	const struct ksz8851_config *config = dev->config;
	struct ksz8851_runtime *ctx = dev->data;
	int ret;
	uint16_t val;
	
	/* Verify chip ID */
	ret = ksz_read16(config, KSZ_REG_CIDER, &val);
	if (ret) {
		LOG_ERR("Failed to read CIDER: %d", ret);
		return ret;
	}
	
	LOG_INF("KSZ8851 CIDER: 0x%04x", val);
	
	if ((val & 0xFFF0) != 0x8870) {
		LOG_ERR("Invalid chip ID: 0x%04x", val);
		return -ENODEV;
	}
	
	/* Enable chip */
	ret = ksz_write16(config, KSZ_REG_CIDER, val | CIDER_ENABLE);
	if (ret) {
		return ret;
	}
	
	/* Soft reset */
	ret = ksz_soft_reset(config);
	if (ret) {
		LOG_ERR("Soft reset failed: %d", ret);
		return ret;
	}

	/* Re-enable chip after soft reset (reset clears enable bit) */
	ret = ksz_read16(config, KSZ_REG_CIDER, &val);
	if (ret) {
		return ret;
	}
	ret = ksz_write16(config, KSZ_REG_CIDER, val | CIDER_ENABLE);
	if (ret) {
		return ret;
	}
	LOG_INF("Chip re-enabled after reset: CIDER=0x%04x", val | CIDER_ENABLE);

	/* Set MAC address */
	ret = ksz_set_mac_address(config, ctx->mac_addr);
	if (ret) {
		LOG_ERR("Failed to set MAC address: %d", ret);
		return ret;
	}
	
	/* Enable QMU transmit frame data pointer auto increment */
	ret = ksz_write16(config, KSZ_REG_TXFDPR, 0x4000);
	if (ret) {
		return ret;
	}
	
	/* Configure RX frame threshold for 1 frame */
	ret = ksz_write16(config, KSZ_REG_RXFCTR, 1);
	if (ret) {
		return ret;
	}
	
	/* Enable QMU RX frame data pointer auto increment */
	ret = ksz_write16(config, KSZ_REG_RXFDPR, 0x4000);
	if (ret) {
		return ret;
	}
	
	/* Configure RX */
	val = RXCR1_RXE |           /* Enable RX */
	      RXCR1_RXFCE |         /* Enable flow control */
	      RXCR1_RXMAFMA;        /* MAC address filtering */
	ret = ksz_write16(config, KSZ_REG_RXCR1, val);
	if (ret) {
		return ret;
	}
	
	/* Configure RX queue: auto-dequeue, single frame threshold */
	val = RXQCR_RXFCTE |        /* Frame count threshold enable */
	      RXQCR_AUTO_DEQUEUE;   /* Auto-dequeue */
	ret = ksz_write16(config, KSZ_REG_RXQCR, val);
	if (ret) {
		return ret;
	}
	
	/* Configure TX */
	val = TXCR_TXE |            /* Enable TX */
	      TXCR_TXFCE;           /* Enable flow control */
	ret = ksz_write16(config, KSZ_REG_TXCR, val);
	if (ret) {
		return ret;
	}
	
	/* Configure PMECR - Enable interrupt pin output (active low) */
	ret = ksz_write16(config, KSZ_REG_PMECR, PMECR_PME_ENABLE);
	if (ret) {
		return ret;
	}
	LOG_INF("PMECR configured for interrupt output");

	/* Clear all interrupts */
	ret = ksz_write16(config, KSZ_REG_ISR, 0xFFFF);
	if (ret) {
		return ret;
	}

	/* Enable interrupts - Try writing multiple times as workaround */
	val = IER_LCIE |            /* Link change */
	      IER_RXIE |            /* RX */
	      IER_TXIE;             /* TX */

	/* First attempt */
	ret = ksz_write16(config, KSZ_REG_IER, val);
	if (ret) {
		LOG_ERR("Failed to write IER (attempt 1): %d", ret);
		return ret;
	}

	/* Verify immediately */
	uint16_t ier_readback = 0;
	ret = ksz_read16(config, KSZ_REG_IER, &ier_readback);
	LOG_INF("IER write attempt 1: wrote 0x%04x, read 0x%04x", val, ier_readback);

	/* If first write failed, try again with a small delay */
	if (ier_readback != val) {
		k_msleep(10);
		LOG_INF("Retrying IER write...");
		ret = ksz_write16(config, KSZ_REG_IER, val);
		if (ret) {
			LOG_ERR("Failed to write IER (attempt 2): %d", ret);
			return ret;
		}
		ret = ksz_read16(config, KSZ_REG_IER, &ier_readback);
		LOG_INF("IER write attempt 2: wrote 0x%04x, read 0x%04x", val, ier_readback);

		if (ier_readback != val) {
			LOG_ERR("IER write failed after 2 attempts! This register may be read-only or locked.");
			LOG_WRN("Falling back to POLLING MODE (50ms interval) for RX");
			/* Start polling timer as fallback */
			k_timer_start(&ctx->poll_timer, K_MSEC(50), K_MSEC(50));
		} else {
			LOG_INF("IER enabled successfully - using interrupt mode");
		}
	} else {
		LOG_INF("IER enabled successfully on first attempt - using interrupt mode");
	}

	LOG_INF("KSZ8851SNL initialized successfully");
	ctx->initialized = true;
	
	return 0;
}

/* RX Path */

/* Poll timer callback - used as fallback when interrupts don't work */
static void ksz_poll_timer_handler(struct k_timer *timer)
{
	struct ksz8851_runtime *ctx = CONTAINER_OF(timer, struct ksz8851_runtime, poll_timer);

	/* Trigger RX work to check for packets */
	k_work_submit(&ctx->rx_work);
}

static void ksz_rx_work_handler(struct k_work *work)
{
	struct ksz8851_runtime *ctx = CONTAINER_OF(work, struct ksz8851_runtime, rx_work);
	const struct device *dev = ctx->dev;
	const struct ksz8851_config *config = dev->config;
	uint16_t frame_count, frame_hdr, frame_len;
	int ret;

	LOG_INF("RX work handler called");

	/* Read RX frame count */
	ret = ksz_read16(config, KSZ_REG_RXFCTR, &frame_count);
	if (ret || !(frame_count & 0xFF)) {
		LOG_INF("RX: No frames available (count=0x%04x, ret=%d)", frame_count, ret);
		goto re_enable_int;
	}

	frame_count = frame_count & 0xFF;
	LOG_INF("RX: %d frame(s) available", frame_count);

	while (frame_count--) {
		/* With auto-dequeue enabled, just read directly from FIFO.
		 * The QMU automatically dequeues the next frame when we read.
		 * Read the 4-byte frame header from FIFO first.
		 */
		uint8_t frame_hdr_buf[4];
		ret = ksz_read_fifo(config, frame_hdr_buf, 4);
		if (ret) {
			LOG_ERR("Failed to read frame header from FIFO: %d", ret);
			break;
		}

		/* Parse frame header:
		 * Byte 0-1: Frame byte count (little endian)
		 * Byte 2-3: Frame status (little endian)
		 */
		frame_len = frame_hdr_buf[0] | (frame_hdr_buf[1] << 8);
		frame_hdr = frame_hdr_buf[2] | (frame_hdr_buf[3] << 8);

		LOG_INF("RX: FIFO header bytes: [0x%02x 0x%02x 0x%02x 0x%02x]",
			frame_hdr_buf[0], frame_hdr_buf[1], frame_hdr_buf[2], frame_hdr_buf[3]);

		/* Mask off the byte count (lower 12 bits) */
		frame_len &= 0x0FFF;

		LOG_INF("RX: Frame status=0x%04x, len=%d", frame_hdr, frame_len);

		/* Check for valid frame (bit 15 = frame valid) */
		if ((frame_hdr & 0x8000) == 0 || frame_len == 0 || frame_len > KSZ_MAX_FRAME_SIZE) {
			LOG_WRN("RX: Invalid frame (hdr=0x%04x, len=%d)", frame_hdr, frame_len);
			ctx->rx_errors++;

			/* Release the error frame */
			ksz_setbits16(config, KSZ_REG_RXQCR, RXQCR_RRXEF);
			continue;
		}
		
		/* Allocate network packet */
		struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, frame_len,
								   AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			LOG_ERR("Failed to allocate RX packet");
			ctx->rx_errors++;
			ksz_setbits16(config, KSZ_REG_RXQCR, RXQCR_RRXEF);
			continue;
		}

		/* Read frame data from FIFO into a temporary buffer */
		uint8_t rx_buf[KSZ_MAX_FRAME_SIZE];
		ret = ksz_read_fifo(config, rx_buf, frame_len);
		if (ret) {
			LOG_ERR("Failed to read frame data: %d", ret);
			net_pkt_unref(pkt);
			ctx->rx_errors++;
			continue;
		}

		/* Write data to packet using proper API */
		ret = net_pkt_write(pkt, rx_buf, frame_len);
		if (ret < 0) {
			LOG_ERR("Failed to write data to packet: %d", ret);
			net_pkt_unref(pkt);
			ctx->rx_errors++;
			continue;
		}
		
		/* Pass packet to network stack */
		ret = net_recv_data(ctx->iface, pkt);
		if (ret < 0) {
			LOG_ERR("Failed to pass packet to stack: %d", ret);
			net_pkt_unref(pkt);
			ctx->rx_errors++;
		} else {
			ctx->rx_packets++;
			LOG_DBG("RX: Packet passed to stack");
		}
	}
	
re_enable_int:
	/* Re-enable interrupts */
	ksz_setbits16(config, KSZ_REG_IER, IER_RXIE);
}

/* TX Path */

static int ksz8851_send(const struct device *dev, struct net_pkt *pkt)
{
	const struct ksz8851_config *config = dev->config;
	struct ksz8851_runtime *ctx = dev->data;
	size_t pkt_len = net_pkt_get_len(pkt);
	int ret;
	uint16_t txmir;
	
	if (!ctx->initialized) {
		LOG_ERR("TX: Device not initialized");
		return -EIO;
	}
	
	if (pkt_len > KSZ_MAX_FRAME_SIZE) {
		LOG_ERR("TX: Packet too large (%zu bytes)", pkt_len);
		return -EMSGSIZE;
	}
	
	LOG_DBG("TX: Sending packet of %zu bytes", pkt_len);
	
	/* Wait for TX buffer space */
	k_sem_take(&ctx->tx_sem, K_FOREVER);
	
	/* Check TX memory available */
	ret = ksz_read16(config, KSZ_REG_TXMIR, &txmir);
	if (ret || (txmir & 0x1FFF) < (pkt_len + 8)) {
		LOG_WRN("TX: Insufficient buffer space (need %zu, have %d)",
			pkt_len + 8, txmir & 0x1FFF);
		k_sem_give(&ctx->tx_sem);
		return -ENOMEM;
	}
	
	/* Enable TXQ write */
	ret = ksz_setbits16(config, KSZ_REG_RXQCR, TXQCR_METFE);
	if (ret) {
		k_sem_give(&ctx->tx_sem);
		return ret;
	}
	
	/* Write TX packet control word (length + interrupt enable) */
	uint8_t ctrl[4];
	ctrl[0] = (pkt_len & 0xFF);
	ctrl[1] = ((pkt_len >> 8) & 0xFF);
	ctrl[2] = 0;  /* Interrupt enable */
	ctrl[3] = 0;
	ret = ksz_write_fifo(config, ctrl, 4);
	if (ret) {
		k_sem_give(&ctx->tx_sem);
		return ret;
	}
	
	/* Write packet data */
	struct net_buf *frag;
	for (frag = pkt->frags; frag; frag = frag->frags) {
		ret = ksz_write_fifo(config, frag->data, frag->len);
		if (ret) {
			k_sem_give(&ctx->tx_sem);
			return ret;
		}
	}
	
	/* Trigger TX (enqueue frame) */
	ret = ksz_setbits16(config, KSZ_REG_TXQCR, TXQCR_METFE);
	if (ret) {
		k_sem_give(&ctx->tx_sem);
		return ret;
	}
	
	ctx->tx_packets++;
	k_sem_give(&ctx->tx_sem);

	LOG_DBG("TX: Packet queued successfully");

	/* Poll ISR to check if any interrupts are pending (diagnostic) */
	uint16_t isr_val = 0, ier_val = 0;
	ksz_read16(config, KSZ_REG_ISR, &isr_val);
	ksz_read16(config, KSZ_REG_IER, &ier_val);
	if (isr_val != 0) {
		LOG_INF("!!! ISR=0x%04x IER=0x%04x (interrupts pending but GPIO not triggered!)",
			isr_val, ier_val);
	}

	return 0;
}

/* Interrupt Handler */

static void ksz_isr_work(const struct device *port, struct gpio_callback *cb,
			 uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	
	struct ksz8851_runtime *ctx = CONTAINER_OF(cb, struct ksz8851_runtime, int_callback);
	const struct device *dev = ctx->dev;
	const struct ksz8851_config *config = dev->config;
	uint16_t isr;
	int ret;
	
	/* Read interrupt status */
	ret = ksz_read16(config, KSZ_REG_ISR, &isr);
	if (ret) {
		return;
	}
	
	if (isr) {
		LOG_INF("ISR: 0x%04x", isr);
	}

	/* Clear interrupts */
	ksz_write16(config, KSZ_REG_ISR, isr);

	/* Handle RX interrupt */
	if (isr & ISR_RXIS) {
		LOG_INF("RX interrupt received");
		/* Disable RX interrupt and schedule work */
		ksz_clrbits16(config, KSZ_REG_IER, IER_RXIE);
		k_work_submit(&ctx->rx_work);
	}

	/* Handle TX interrupt */
	if (isr & ISR_TXIS) {
		LOG_DBG("TX interrupt received");
	}
	
	/* Handle link change */
	if (isr & ISR_LCIS) {
		uint16_t status;
		ret = ksz_read16(config, KSZ_REG_P1SR, &status);
		if (!ret) {
			bool link = (status & P1SR_LINK_GOOD) != 0;
			if (link != ctx->link_up) {
				ctx->link_up = link;
				LOG_INF("Link %s", link ? "UP" : "DOWN");
				
				if (link) {
					net_eth_carrier_on(ctx->iface);
				} else {
					net_eth_carrier_off(ctx->iface);
				}
			}
		}
	}
}

/* Network Interface API */

static enum ethernet_hw_caps ksz8851_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T;
}

static int ksz8851_start(const struct device *dev)
{
	struct ksz8851_runtime *ctx = dev->data;
	
	LOG_INF("Starting interface");
	
	if (!ctx->initialized) {
		return ksz_init_chip(dev);
	}
	
	return 0;
}

static int ksz8851_stop(const struct device *dev)
{
	const struct ksz8851_config *config = dev->config;
	
	LOG_INF("Stopping interface");
	
	/* Disable RX/TX */
	ksz_clrbits16(config, KSZ_REG_RXCR1, RXCR1_RXE);
	ksz_clrbits16(config, KSZ_REG_TXCR, TXCR_TXE);
	
	return 0;
}

static void ksz8851_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct ksz8851_runtime *ctx = dev->data;
	
	ctx->iface = iface;
	
	/* Set MAC address */
	net_if_set_link_addr(iface, ctx->mac_addr, sizeof(ctx->mac_addr),
			     NET_LINK_ETHERNET);
	
	ethernet_init(iface);
	
	LOG_INF("Interface initialized");
}

static const struct ethernet_api ksz8851_api = {
	.iface_api.init = ksz8851_iface_init,
	.get_capabilities = ksz8851_get_capabilities,
	.send = ksz8851_send,
	.start = ksz8851_start,
	.stop = ksz8851_stop,
};

/* Device Initialization */

static int ksz8851_init(const struct device *dev)
{
	const struct ksz8851_config *config = dev->config;
	struct ksz8851_runtime *ctx = dev->data;
	int ret;
	
	/* Small delay to let UART come up so we can see logs */
	k_msleep(100);

	LOG_INF("==== KSZ8851SNL driver init called ====");
	LOG_INF("Device: %s", dev->name);

	ctx->dev = dev;
	
	/* Initialize semaphore */
	k_sem_init(&ctx->tx_sem, 1, 1);

	/* Initialize work queue for RX */
	k_work_init(&ctx->rx_work, ksz_rx_work_handler);

	/* Initialize polling timer (50ms poll rate as fallback) */
	k_timer_init(&ctx->poll_timer, ksz_poll_timer_handler, NULL);
	
	/* Generate MAC address (modify as needed) */
	ctx->mac_addr[0] = 0x00;
	ctx->mac_addr[1] = 0x10;
	ctx->mac_addr[2] = 0x20;
	ctx->mac_addr[3] = 0x30;
	ctx->mac_addr[4] = 0x40;
	ctx->mac_addr[5] = 0x50;
	
	/* Check SPI bus ready */
	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}
	
	/* Configure reset GPIO */
	if (config->reset.port) {
		if (!device_is_ready(config->reset.port)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}
		
		ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
		if (ret) {
			return ret;
		}
		
		/* Reset sequence */
		gpio_pin_set_dt(&config->reset, 1);  /* Assert reset */
		k_msleep(10);
		gpio_pin_set_dt(&config->reset, 0);  /* Release reset */
		k_msleep(100);
		
		LOG_INF("Reset sequence completed");
	}
	
	/* Configure interrupt GPIO */
	if (config->interrupt.port) {
		if (!device_is_ready(config->interrupt.port)) {
			LOG_ERR("Interrupt GPIO not ready");
			return -ENODEV;
		}
		
		ret = gpio_pin_configure_dt(&config->interrupt, GPIO_INPUT);
		if (ret) {
			return ret;
		}
		
		ret = gpio_pin_interrupt_configure_dt(&config->interrupt,
						     GPIO_INT_EDGE_FALLING);
		if (ret) {
			return ret;
		}
		
		gpio_init_callback(&ctx->int_callback, ksz_isr_work,
				  BIT(config->interrupt.pin));
		
		ret = gpio_add_callback(config->interrupt.port, &ctx->int_callback);
		if (ret) {
			return ret;
		}
		
		LOG_INF("Interrupt configured");
	}
	
	/* Initialize chip */
	ret = ksz_init_chip(dev);
	if (ret) {
		LOG_ERR("Chip initialization failed: %d", ret);
		return ret;
	}
	
	LOG_INF("KSZ8851SNL driver initialized");
	return 0;
}

/* Device instantiation */

#define KSZ8851_DEFINE(inst)								\
	static struct ksz8851_runtime ksz8851_data_##inst;				\
											\
	static const struct ksz8851_config ksz8851_config_##inst = {			\
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8), 0),			\
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),		\
		.interrupt = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),		\
	};										\
											\
	ETH_NET_DEVICE_DT_INST_DEFINE(inst,						\
				     ksz8851_init,					\
				     NULL,						\
				     &ksz8851_data_##inst,				\
				     &ksz8851_config_##inst,				\
				     CONFIG_ETH_INIT_PRIORITY,				\
				     &ksz8851_api,					\
				     NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(KSZ8851_DEFINE)
