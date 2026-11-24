#define DT_DRV_COMPAT murata_scl3300

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/rtio/iodev.h>
#include <zephyr/rtio/sqe.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "scl3300.h"

LOG_MODULE_REGISTER(scl3300, CONFIG_SENSOR_LOG_LEVEL);

SPI_DT_IODEV_DEFINE(scl3300_spi_iodev, DT_DRV_INST(0), SPI_WORD_SET(8) | SPI_TRANSFER_MSB);

static const struct scl3300_config scl3300_config = {
	.spi_iodev = &scl3300_spi_iodev,
	.bus_init = scl3300_spi_init,
	.init = DT_INST_PROP_OR(0, init, 0),
};

RTIO_DEFINE_WITH_MEMPOOL(scl3300_rtio, 2, /* Submission queue depth */
			 2,               /* Completion queue depth */
			 2,               /* Number of memory blocks */
			 256,             /* Memory block size */
			 sizeof(void *)   /* Memory block alignment */
);

static struct scl3300_data scl3300_data = {
	.rtio = &scl3300_rtio,
};

static uint8_t scl3300_crc8_update(uint8_t bit_val, uint8_t crc)
{
	crc ^= bit_val;
	for (int i = 8; i > 0; i--) {
		crc = (crc & 0x80) ? (crc << 1) ^ 0x1D : (crc << 1);
	}
	return crc;
}

static bool scl3300_validate_crc(const uint8_t *rx_buf)
{
	uint32_t frame = ((uint32_t)rx_buf[0] << 24) | ((uint32_t)rx_buf[1] << 16) |
			 ((uint32_t)rx_buf[2] << 8) | (uint32_t)rx_buf[3];

	uint8_t crc = 0xFF;

	crc = scl3300_crc8_update((frame >> 24) & 0xFF, crc);
	crc = scl3300_crc8_update((frame >> 16) & 0xFF, crc);
	crc = scl3300_crc8_update((frame >> 8) & 0xFF, crc);

	return ((~crc & 0xFF) == (frame & 0xFF));
}

static int scl3300_read_angle_response(const struct device *dev, uint32_t command, uint8_t *rx_buf)
{
	struct scl3300_data *data = dev->data;
	uint8_t dummy_rx[4];
	int ret;

	/* First frame: send the requested angle command. */
	ret = scl3300_transport_read(data->ctx, command, dummy_rx);
	if (ret != 0) {
		return ret;
	}

	/* Second frame: clock out the angle response. */
	ret = scl3300_transport_read(data->ctx, SCL3300_CMD_STATUS, rx_buf);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int scl3300_fetch_angle(const struct device *dev, uint32_t command)
{
	struct scl3300_data *data = dev->data;
	uint8_t rx_buf[SCL3300_SPI_FRAME_SIZE];

	scl3300_read_angle_response(dev, command, rx_buf);

	if (!scl3300_validate_crc(rx_buf)) {
		LOG_ERR("CRC validation failed");
		return -EIO;
	}

	if (command == SCL3300_CMD_READ_ANG_X) {
		data->angle_x = (int16_t)(((uint16_t)rx_buf[1] << 8) | rx_buf[2]);
	} else if (command == SCL3300_CMD_READ_ANG_Y) {
		data->angle_y = (int16_t)(((uint16_t)rx_buf[1] << 8) | rx_buf[2]);
	} else if (command == SCL3300_CMD_READ_ANG_Z) {
		data->angle_z = (int16_t)(((uint16_t)rx_buf[1] << 8) | rx_buf[2]);
	}

	return 0;
}

static int scl3300_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret = 0;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		ret = scl3300_fetch_angle(dev, SCL3300_CMD_READ_ANG_X);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		ret = scl3300_fetch_angle(dev, SCL3300_CMD_READ_ANG_Y);
	case SENSOR_CHAN_ACCEL_Z:
		ret = scl3300_fetch_angle(dev, SCL3300_CMD_READ_ANG_Z);
	case SENSOR_CHAN_ACCEL_XYZ:
		ret = scl3300_fetch_angle(dev, SCL3300_CMD_READ_ANG_X);
		if (ret != 0) {
			break;
		}

		ret = scl3300_fetch_angle(dev, SCL3300_CMD_READ_ANG_Y);
		if (ret != 0) {
			break;
		}

		ret = scl3300_fetch_angle(dev, SCL3300_CMD_READ_ANG_Z);
		if (ret != 0) {
			break;
		}

	default:
		return -ENOTSUP;
	}

	return ret;
}

static int scl3300_configure(const struct device *dev, const struct scl3300_config *cfg)
{
	struct scl3300_data *data = dev->data;
	int ret;

	if ((data == NULL) || (data->ctx == NULL)) {
		LOG_ERR("SCL3300 context is not initialized");
		return -EINVAL;
	}

	ret = scl3300_transport_write(data->ctx, SCL3300_CMD_WHOAMI);
	if (ret < 0) {
		LOG_ERR("Failed to send WHOAMI command: %d", ret);
		return ret;
	}

	if (cfg->init) {
		ret = scl3300_transport_write(data->ctx, SCL3300_CMD_WAKEUP);
		if (ret < 0) {
			LOG_ERR("Failed to send WAKEUP command: %d", ret);
			return ret;
		}

		ret = scl3300_transport_write(data->ctx, SCL3300_CMD_SW_RESET);
		if (ret < 0) {
			LOG_ERR("Failed to send SW_RESET command: %d", ret);
			return ret;
		}

		k_msleep(2);

		ret = scl3300_transport_write(data->ctx, SCL3300_CMD_ANG_CTRL);
		if (ret < 0) {
			LOG_ERR("Failed to send ANG_CTRL command: %d", ret);
			return ret;
		}

		k_msleep(25);

		ret = scl3300_transport_write(data->ctx, SCL3300_CMD_STATUS);
		if (ret < 0) {
			LOG_ERR("Failed to send STATUS command: %d", ret);
			return ret;
		}

		ret = scl3300_transport_write(data->ctx, SCL3300_CMD_STATUS);
		if (ret < 0) {
			LOG_ERR("Failed to send STATUS command: %d", ret);
			return ret;
		}

		ret = scl3300_transport_write(data->ctx, SCL3300_CMD_STATUS);
		if (ret < 0) {
			LOG_ERR("Failed to send STATUS command: %d", ret);
			return ret;
		}
	}

	/* TODO: Runtime measurement mode configuration. */

	return ret;
}

static int scl3300_safely_configure(const struct device *dev, const struct scl3300_config *cfg)
{
	if ((dev == NULL) || (cfg == NULL) || (dev->config == NULL)) {
		return -EINVAL;
	}

	/* TODO: Perform validation and backup. */

	return scl3300_configure(dev, cfg);
}

static int scl3300_init_device(const struct device *dev)
{
	const struct scl3300_config *cfg = dev->config;
	int ret;

	if (!spi_is_ready_iodev(cfg->spi_iodev)) {
		LOG_ERR("SCL3300 SPI bus is not ready");
		return -ENODEV;
	}

	ret = cfg->bus_init(dev);
	if (ret < 0) {
		LOG_ERR("SCL3300 SPI initialization failed: %d", ret);
		return ret;
	}

	ret = scl3300_safely_configure(dev, cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure SCL3300: %d", ret);
		return ret;
	}

	return ret;
}

static DEVICE_API(sensor, scl3300_driver_api) = {
	.sample_fetch = scl3300_sample_fetch,
};

DEVICE_DT_INST_DEFINE(0, scl3300_init_device, NULL, &scl3300_data, &scl3300_config, POST_KERNEL,
		      CONFIG_SENSOR_INIT_PRIORITY, &scl3300_driver_api);
