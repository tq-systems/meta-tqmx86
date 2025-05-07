// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * edid-tool: write edid data to PTN3460 E(DP) to LVDS Bridge
 *
 * Copyright (c) 2021-2024 TQ-Systems GmbH <oss@ew.tq-group.com>, D-82229 Seefeld, Germany.
 * Author: Gregor Herburger
 */

#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/stat.h>

#include "i2cbusses.h"

#define EDID_LENGTH 128

#define REG_EDID	0x0
#define REG_EDID_ROM_NR 0x85
#define REG_FLASH_CMD	0xE8

#define ADAPTER_NAME "i2c-machxo2"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static int read_i2c_byte(int fd_bus, uint8_t addr, uint8_t offset)
{
	struct i2c_rdwr_ioctl_data msg;
	struct i2c_msg data[2];
	uint8_t outbuf[1], inbuf[4];
	int ret;

	outbuf[0] = offset;

	data[0].addr = addr;
	data[0].flags = 0;
	data[0].len = sizeof(outbuf);
	data[0].buf = outbuf;

	data[1].addr = addr;
	data[1].flags = I2C_M_RD | I2C_M_STOP;
	data[1].len = sizeof(inbuf);
	data[1].buf = inbuf;

	msg.msgs = data;
	msg.nmsgs = ARRAY_SIZE(data);

	ret = ioctl(fd_bus, I2C_RDWR, &msg);
	if (ret < 0)
		return ret;

	return inbuf[0];
}

static int write_i2c_block(int fd_bus, uint8_t offset, uint8_t *buf, size_t len)
{
	int ret;
	uint8_t *writebuf;

	writebuf = malloc(len + 1);
	writebuf[0] = offset;
	memcpy(&writebuf[1], buf, len);

	ret = write(fd_bus, writebuf, len + 1);
	if (ret != len + 1) {
		fprintf(stderr, "Failed to write %ld bytes: %s: %d\n", len, strerror(errno), ret);
		ret = 1;
	} else
		ret = 0;

	free(writebuf);
	return ret;
}

static int write_i2c_byte(int fd_bus, uint8_t offset, uint8_t value)
{
	return write_i2c_block(fd_bus, offset, &value, 1);
}

static int write_edid_data_to_sram(int fd_bus, uint8_t edidnum, uint8_t *buf)
{
	int ret;

	ret = write_i2c_byte(fd_bus, REG_EDID_ROM_NR, edidnum);
	if (ret) {
		fprintf(stderr, "Failed to write edidnum: %s(%d)\n", strerror(errno), ret);
		return 1;
	}

	ret = write_i2c_block(fd_bus, REG_EDID, buf, EDID_LENGTH);
	if (ret)
		return 2;

	return 0;
}

static int write_edid_data_to_flash(int fd_bus, uint8_t edidnum, uint8_t *buf)
{
	int ret;
	uint8_t write_flash[] = {0x01, 0x78, 0x45, 0x56};

	ret = write_edid_data_to_sram(fd_bus, edidnum, buf);
	if (ret) {
		fprintf(stderr, "Failed to write edidnum: %s(%d)\n", strerror(errno), ret);
		return 1;
	}

	ret = write_i2c_block(fd_bus, REG_FLASH_CMD, write_flash, ARRAY_SIZE(write_flash));
	if (ret)
		return 2;

	return 0;
}

static int find_ptn3460_bus(uint8_t addr)
{
	struct i2c_adap *adapters;
	int ret = -1;
	int count;
	int bus;
	char filename_bus[20];
	int val;

	adapters = gather_i2c_busses();
	if (adapters == NULL) {
		fprintf(stderr, "Error: Out of memory!\n");
		return -1;
	}

	for (count = 0; adapters[count].name; count++) {
		if (strcmp(ADAPTER_NAME, adapters[count].name))
			continue;

		snprintf(filename_bus, 20, "/dev/i2c-%d", adapters[count].nr);
		bus = open(filename_bus, O_RDWR);
		if (bus < 0)
			continue;

		if (ioctl(bus, I2C_SLAVE, addr)) {
			close(bus);
			continue;
		}

		val = read_i2c_byte(bus, addr, REG_EDID_ROM_NR);
		close(bus);
		if (val < 0 && val > 6)
			continue;

		ret = adapters[count].nr;
		break;
	}

	free_adapters(adapters);
	return ret;
}

static char *helptext =
	"Usage: edid-tool [OPTIONS] FILENAME\n"
	"Write EDID-data from filename to ptn3460 flash memory\n"
	"Options:\n"
	" -b	Bus number. If none try to find bus.\n"
	" -e	EDID number (0-6), default 6\n"
	" -a	Address 0x40 or 0x60, default 0x60";

static void print_help(void)
{
	puts(helptext);
}

int main(int argc, char *argv[])
{
	int opt;
	int busnum = 0;
	int edidnum = -1;
	char *filename = NULL;
	FILE *fd_edid;
	int fd_bus;
	int addr = 0x60;
	struct stat statbuf;
	char filename_i2c[20];
	uint8_t edid_buf[EDID_LENGTH];
	int ret;

	while ((opt = getopt(argc, argv, "b:e:a:h")) != -1) {
		switch (opt) {
		case 'b':
			busnum = atoi(optarg);
			break;
		case 'e':
			edidnum = atoi(optarg);
			if (edidnum > 6 || edidnum < 0) {
				printf("edidnum not correct: %d\n", edidnum);
				print_help();
				exit(1);
			}
			break;
		case 'a':
			addr = strtol(optarg, NULL, 16);
			if (!(addr == 0x60 || addr == 0x40)) {
				printf("Address not valid: 0x%02x\n", addr);
				print_help();
				exit(1);
			}
			break;
		case 'h':
			print_help();
			exit(0);
			break;
		default:
			printf("Error: %d\n", opt);
			print_help();
			exit(1);
			break;
		}
	}

	if (optind + 1 != argc) {
		printf("Wrong argument count.\n");
		print_help();
		exit(1);
	}

	filename = argv[optind];
	fd_edid = fopen(filename, "r");
	if (!fd_edid) {
		printf("File not found: %s\n", filename);
		goto error_and_exit;
	}

	if (fstat(fileno(fd_edid), &statbuf) < 0)
		goto error_edid;

	if (statbuf.st_size > EDID_LENGTH)
		goto error_edid;

	ret = fread(edid_buf, 1, EDID_LENGTH, fd_edid);
	if (ret != EDID_LENGTH) {
		fprintf(stderr, "Error: Could not read edid data from file: %d.", ret);
		goto error_edid;
	}

	if (!busnum) {
		busnum = find_ptn3460_bus(addr);
		if (busnum <= 0) {
			fprintf(stderr, "Error to find bus: %s(%d)\n", strerror(errno), busnum);
			goto error_edid;
		} else
			printf("Found Bus: %d\n", busnum);
	}

	if (edidnum == -1) {
		edidnum = 6;
		printf("Using default EDID %d\n", edidnum);
	}

	snprintf(filename_i2c, 20, "/dev/i2c-%d", busnum);
	fd_bus = open(filename_i2c, O_RDWR);

	if (fd_bus < 0) {
		fprintf(stderr, "Error: Failed to open %s: %s\n", filename_i2c, strerror(errno));
		goto error_edid;
	}

	ret = ioctl(fd_bus, I2C_SLAVE, addr);
	if (ret) {
		fprintf(stderr, "Error: Could not set addr 0x%02x: %s\n", addr, strerror(errno));
		goto error_bus;
	}

	ret = write_edid_data_to_flash(fd_bus, edidnum, edid_buf);
	if (ret)
		goto error_bus;

	printf("File successfully written.\n");

	fclose(fd_edid);
	close(fd_bus);
	return 0;

error_bus:
	close(fd_bus);
error_edid:
	fclose(fd_edid);
error_and_exit:
	exit(2);
}
