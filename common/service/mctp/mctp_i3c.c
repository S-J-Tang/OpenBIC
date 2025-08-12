/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "plat_def.h"
#ifdef ENABLE_MCTP_I3C
#include "mctp.h"

#include <stdlib.h>
#include <string.h>
#include <zephyr.h>
#include <sys/crc.h>
#include <logging/log.h>
#include <sys/ring_buffer.h>
#include "libutil.h"
#include "hal_i3c.h"
#include "mctp_ctrl.h"
#include "plat_gpio.h"

LOG_MODULE_REGISTER(mctp_i3c);

#ifndef MCTP_I3C_PEC_ENABLE
#define MCTP_I3C_PEC_ENABLE 0
#endif

#ifndef MCTP_I3C_MULTIPLE_PACKAGES_ENABLE
#define MCTP_I3C_MULTIPLE_PACKAGES_ENABLE 0
#endif

#if MCTP_I3C_MULTIPLE_PACKAGES_ENABLE
/* MCTP Header */
typedef struct __attribute__((packed)) {
       uint8_t hdr_ver;
       uint8_t dest_ep;
       uint8_t src_ep;
       union {
               struct {
                       uint8_t msg_tag : 3;
                       uint8_t to : 1;
                       uint8_t pkt_seq : 2;
                       uint8_t eom : 1;
                       uint8_t som : 1;
               };
               uint8_t flags_seq_to_tag;
       };
} mctp_hdr;

/* State tracking for SOM/EOM validation per message tag */
typedef struct {
       bool som_received[8];
} mctp_som_eom_state_t;

static mctp_som_eom_state_t som_eom_state = {0};

/* MCTP I3C units */
#define MCTP_I3C_MAX_PAYLOAD_SIZE MCTP_BASE_LINE_UNIT
#define MCTP_I3C_FIXED_PACKAGE_SIZE                                                                \
       MCTP_I3C_MAX_PAYLOAD_SIZE + sizeof(mctp_hdr) + (MCTP_I3C_PEC_ENABLE ? 1 : 0)

/* MCTP I3C ring buffer - size = I3C_MAX_DATA_SIZE * 2 = 512 = 2^9 */
RING_BUF_ITEM_DECLARE_POW2(mctp_i3c_smq_ringbuf, 9);

/* MCTP header pattern */
typedef struct {
       uint8_t som_eom;
       bool requires_fixed_length;
       bool requires_seq_range_check;
       uint8_t min_seq;
       uint8_t max_seq;
} mctp_header_pattern_t;

static const mctp_header_pattern_t mctp_header_table[] = {
       { 0b11, false, false, 0, 0 }, // SOM=1, EOM=1 - Single package
       { 0b01, false, false, 0, 0 }, // SOM=0, EOM=1 - Last package
       { 0b10, true, false, 0, 0 }, // SOM=1, EOM=0 - First package
       { 0b00, true, true, 1, 3 }, // SOM=0, EOM=0 - Middle package
};
#endif

static uint16_t mctp_i3c_read(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params *extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(extra_data, MCTP_ERROR);

	mctp *mctp_inst = (mctp *)mctp_p;
	I3C_MSG i3c_msg = { 0 };

	i3c_msg.bus = mctp_inst->medium_conf.i3c_conf.bus;
	i3c_msg.target_addr = mctp_inst->medium_conf.i3c_conf.addr;

	int ret = i3c_controller_ibi_read(&i3c_msg);

	/** mctp rx keep polling, return length 0 directly if no data or invalid data **/
	if (ret <= 0) {
		return 0;
	}

	i3c_msg.rx_len = ret;
	LOG_HEXDUMP_DBG(&i3c_msg.data[0], i3c_msg.rx_len, "mctp_i3c_read_smq msg dump");

	if (MCTP_I3C_PEC_ENABLE) {
		uint8_t pec = 0x0, dynamic_addr = 0x0;

		/** pec byte use 7-degree polynomial with 0 init value and false reverse **/
		dynamic_addr = i3c_msg.target_addr << 1 | 1;
		pec = crc8(&dynamic_addr, 1, 0x07, 0x00, false);
		pec = crc8(&i3c_msg.data[0], i3c_msg.rx_len - 1, 0x07, pec, false);
		if (pec != i3c_msg.data[i3c_msg.rx_len - 1]) {
			LOG_ERR("mctp i3c pec error: crc8 should be 0x%02x, but got 0x%02x", pec,
				i3c_msg.data[i3c_msg.rx_len - 1]);
			return 0;
		}
		/** Remove pec byte if it is valid **/
		i3c_msg.rx_len--;
	}

	extra_data->type = MCTP_MEDIUM_TYPE_CONTROLLER_I3C;
	memcpy(buf, &i3c_msg.data[0], i3c_msg.rx_len);
	return i3c_msg.rx_len;
}

static uint16_t mctp_i3c_write(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);

	if (extra_data.type != MCTP_MEDIUM_TYPE_CONTROLLER_I3C) {
		LOG_ERR("mctp medium type incorrect");
		return MCTP_ERROR;
	}

	int ret;
	I3C_MSG i3c_msg;
	mctp *mctp_instance = (mctp *)mctp_p;

	i3c_msg.bus = mctp_instance->medium_conf.i3c_conf.bus;
	i3c_msg.target_addr = mctp_instance->medium_conf.i3c_conf.addr;

	/** mctp package **/
	if (len < I3C_MAX_DATA_SIZE) {
		memcpy(&i3c_msg.data[0], buf, len);
	} else {
		LOG_ERR("Write failed because I3C data length exceed %d bytes", I3C_MAX_DATA_SIZE);
		return MCTP_ERROR;
	}

	/** +1 pec; default no pec **/
	if (MCTP_I3C_PEC_ENABLE) {
		uint8_t pec = 0x0, dynamic_addr = 0x0;

		i3c_msg.tx_len = len + 1;
		/** pec byte use 7-degree polynomial with 0 init value and false reverse **/
		dynamic_addr = i3c_msg.target_addr << 1;
		pec = crc8(&dynamic_addr, 1, 0x07, 0x00, false);
		pec = crc8(&i3c_msg.data[0], len, 0x07, pec, false);
		i3c_msg.data[len] = pec;
	} else {
		i3c_msg.tx_len = len;
	}

	LOG_HEXDUMP_DBG(&i3c_msg.data[0], i3c_msg.tx_len, "mctp_i3c_write msg dump");

	ret = i3c_controller_write(&i3c_msg);
	if (ret < 0) {
		LOG_ERR("i3c controller write failed, %d", ret);
		return MCTP_ERROR;
	}
	return MCTP_SUCCESS;
}
#if MCTP_I3C_MULTIPLE_PACKAGES_ENABLE
static const mctp_header_pattern_t *get_mctp_header_pattern(uint8_t som_eom)
{
       for (size_t i = 0; i < ARRAY_SIZE(mctp_header_table); i++) {
               if (mctp_header_table[i].som_eom == som_eom) {
                       return &mctp_header_table[i];
               }
       }
       return NULL;
}

static uint16_t mctp_i3c_smq_ringbuf_read(uint8_t *buf, uint32_t len, mctp_ext_params *extra_data)
{
       uint16_t read_len;
       uint16_t type;
       uint8_t value;
       uint8_t size32;
       uint16_t package_len;
       int ret;

       /* Buffer availability check */
       if (ring_buf_space_get(&mctp_i3c_smq_ringbuf) ==
           ring_buf_capacity_get(&mctp_i3c_smq_ringbuf)) {
               return 0;
       }

       /* Read with length information */
       ret = ring_buf_item_get(&mctp_i3c_smq_ringbuf, &type, &value, (uint32_t *)buf, &size32);
       if (ret != 0) {
               return 0;
       }

       /* Calculate actual byte length from 32-bit word count minus padding */
       package_len = (size32 << 2) - value;
       read_len = package_len;
       if (read_len > len) {
               LOG_WRN("Package length %d exceeds buffer size %d", read_len, len);
               read_len = len;
       }

       /* Fill remaining buffer space with 0xff padding if package is shorter than requested length */
       if (package_len < len) {
               memset(buf + package_len, 0xff, len - package_len);
       }

       if (read_len > 0) {
               extra_data->type = MCTP_MEDIUM_TYPE_TARGET_I3C;
       }

       return read_len;
}

static bool validate_package_length(mctp_hdr *hdr, uint16_t package_len)
{
       uint16_t expected_fixed_len = MCTP_I3C_FIXED_PACKAGE_SIZE;
       const mctp_header_pattern_t *pattern = get_mctp_header_pattern((hdr->som << 1) | hdr->eom);

       if (!pattern) {
               return false;
      }

       if (pattern->requires_fixed_length) {
               if (package_len != expected_fixed_len) {
                       LOG_WRN("Package length %d should be %d (SOM=%d, EOM=%d)", package_len,
                               expected_fixed_len, hdr->som, hdr->eom);
                       return false;
               }
       }

       if (pattern->requires_seq_range_check) {
               if (hdr->pkt_seq < pattern->min_seq || hdr->pkt_seq > pattern->max_seq) {
                       if (pattern->requires_fixed_length && package_len != expected_fixed_len) {
                               LOG_WRN("Middle package length %d should be %d (SOM=0, EOM=0, Pkt_Seq=%d)",
                                       package_len, expected_fixed_len, hdr->pkt_seq);
                               return false;
                       }
               }
       }

       return true;
}

static bool validate_package_pec(uint8_t *package_data, uint16_t package_len, uint8_t dynamic_addr)
{
       if (!MCTP_I3C_PEC_ENABLE) {
               return true;
       }

       if (package_len < 1) {
               LOG_ERR("Package too short for PEC validation");
               return false;
       }

       uint8_t pec = 0x0;
       uint8_t addr_byte = dynamic_addr << 1;

       pec = crc8(&addr_byte, 1, 0x07, 0x00, false);
       pec = crc8(package_data, package_len - 1, 0x07, pec, false);

       if (pec != package_data[package_len - 1]) {
               LOG_ERR("0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, %d", package_data[0],
                       package_data[1], package_data[2], package_data[3], package_data[4],
                       package_data[package_len - 1], package_len);
               LOG_ERR("mctp i3c pec error: crc8 should be 0x%02x, but got 0x%02x", pec,
                       package_data[package_len - 1]);
               bool current_state = gpio_get(TP_NPCM_GPIOC6);
               gpio_set(TP_NPCM_GPIOC6, !current_state);
               return false;
       }

       return true;
}

__weak bool plat_i3c_is_valid_endpoint_id(uint8_t dest_ep, uint8_t src_ep)
{
       (void)dest_ep;
       (void)src_ep;
       return true;
}

static bool validate_som_eom_sequence(mctp_hdr *hdr)
{
       uint8_t msg_tag = hdr->msg_tag;
       bool som = hdr->som;
       bool eom = hdr->eom;

       if (som && eom) {
               return true;
       }

       if (som && !eom) {
               if (som_eom_state.som_received[msg_tag]) {
                       LOG_WRN("SOM detected after earlier SOM without EOM for msg_tag %d",
                               msg_tag);
                       return false;
               }
               som_eom_state.som_received[msg_tag] = true;
               return true;
       }

       if (!som && eom) {
               if (!som_eom_state.som_received[msg_tag]) {
                       LOG_WRN("EOM detected without earlier SOM for msg_tag %d", msg_tag);
                       return false;
               }
               som_eom_state.som_received[msg_tag] = false;
               return true;
       }

       if (!som && !eom) {
               if (!som_eom_state.som_received[msg_tag]) {
                       LOG_WRN("Middle packet detected without earlier SOM for msg_tag %d",
                               msg_tag);
                       return false;
               }
               return true;
       }

       return true;
}

static bool is_valid_mctp_header(mctp_hdr *hdr, uint16_t remaining_len)
{
       if (hdr->hdr_ver != MCTP_HDR_HDR_VER || hdr->pkt_seq > 3) {
               return false;
       }

       if (!plat_i3c_is_valid_endpoint_id(hdr->dest_ep, hdr->src_ep)) {
               return false;
       }

       if (!validate_som_eom_sequence(hdr)) {
               return false;
       }

       const mctp_header_pattern_t *pattern = get_mctp_header_pattern((hdr->som << 1) | hdr->eom);

       if (!pattern) {
               return false;
       }

       if (pattern->requires_seq_range_check) {
               if (hdr->pkt_seq < pattern->min_seq || hdr->pkt_seq > pattern->max_seq) {
                       return false;
               }
       }

       if (pattern->requires_fixed_length) {
               return (remaining_len >= MCTP_I3C_FIXED_PACKAGE_SIZE);
       }

       return true;
}

static uint16_t calculate_package_payload_length(uint8_t *data, uint16_t pkg_start,
                                                uint16_t total_len, uint16_t max_payload_size)
{
       mctp_hdr *hdr = (mctp_hdr *)(data + pkg_start);
       uint16_t payload_start = pkg_start + sizeof(mctp_hdr);
       uint16_t remaining_data = total_len - payload_start;
       uint16_t pec_size = MCTP_I3C_PEC_ENABLE ? 1 : 0;
       const mctp_header_pattern_t *pattern = get_mctp_header_pattern((hdr->som << 1) | hdr->eom);
       uint16_t search_start;
       uint16_t search_pos;
       mctp_hdr *next_hdr;
       uint16_t remaining_from_pos;
       uint16_t total_payload_and_pec;

       if (pattern && pattern->requires_fixed_length) {
               return MCTP_I3C_MAX_PAYLOAD_SIZE;
       }

       /* Variable length packages (Single or Last) - need to search for next header */
       search_start = payload_start + sizeof(mctp_hdr) + pec_size;
       for (search_pos = search_start; search_pos + sizeof(mctp_hdr) <= total_len;
            search_pos += sizeof(mctp_hdr)) {
               next_hdr = (mctp_hdr *)(data + search_pos);
               remaining_from_pos = total_len - search_pos;
               if (is_valid_mctp_header(next_hdr, remaining_from_pos)) {
                       total_payload_and_pec = search_pos - payload_start;
                       return (total_payload_and_pec > pec_size) ?
                                      total_payload_and_pec - pec_size :
                                      0;
               }
       }

       return (remaining_data > pec_size) ? remaining_data - pec_size : remaining_data;
}

static uint16_t queue_multiple_packages(uint8_t *raw_data, uint16_t raw_len,
                                                   uint16_t max_msg_size, uint8_t dynamic_addr)
{
       uint16_t package_offset = 0;
       uint16_t packages_queued = 0;
       mctp_hdr *hdr;
       uint16_t remaining_len;
       uint16_t payload_len;
       uint16_t pec_size = MCTP_I3C_PEC_ENABLE ? 1 : 0;
       uint16_t current_package_len;
       uint8_t *package_data;
       uint16_t package_len_without_pec;
       uint8_t size32;
       uint8_t padding;
       int wrote;

       while (package_offset + sizeof(mctp_hdr) <= raw_len) {
               hdr = (mctp_hdr *)(raw_data + package_offset);
               remaining_len = raw_len - package_offset;

               if (!is_valid_mctp_header(hdr, remaining_len)) {
                       LOG_WRN("Invalid MCTP header at offset %d: ver=0x%02x, seq=%d, som=%d, eom=%d, remaining=%d",
                               package_offset, hdr->hdr_ver, hdr->pkt_seq, hdr->som, hdr->eom,
                               remaining_len);
                       break;
               }

               payload_len = calculate_package_payload_length(raw_data, package_offset, raw_len,
                                                              max_msg_size);
               current_package_len = sizeof(mctp_hdr) + payload_len + pec_size;
               LOG_DBG("payload_len=%d, current_package_len=%d, raw_len=%d, package_offset=%d\n",
                      payload_len, current_package_len, raw_len, package_offset);

               if (package_offset + current_package_len > raw_len) {
                       LOG_WRN("Package length %d exceeds available data at offset %d",
                               current_package_len, package_offset);
                       break;
               }

               package_data = raw_data + package_offset;

               if (!validate_package_length(hdr, current_package_len)) {
                       LOG_WRN("Package %d length validation failed, skipping",
                               packages_queued + 1);
                       package_offset += current_package_len;
                       continue;
               }

               if (!validate_package_pec(package_data, current_package_len, dynamic_addr)) {
                       LOG_WRN("Package %d PEC validation failed, skipping", packages_queued + 1);
                       package_offset += current_package_len;
                       continue;
               }

               package_len_without_pec = sizeof(mctp_hdr) + payload_len;

               /* Convert byte length to 32-bit word count */
               size32 = (package_len_without_pec + 3) >> 2;
               padding = (size32 << 2) - package_len_without_pec;
               wrote = ring_buf_item_put(&mctp_i3c_smq_ringbuf, 0, padding,
                                         (uint32_t *)package_data, size32);
               if (wrote != 0) {
                       LOG_WRN("Ring buffer full, failed to store package %d (len=%d)",
                               packages_queued + 1, package_len_without_pec);
               } else {
                       packages_queued++;
                       LOG_DBG("Queued package %d: offset=%d, len=%d (without PEC), SOM=%d, EOM=%d pkt_seq=%d",
                               packages_queued, package_offset, package_len_without_pec, hdr->som,
                               hdr->eom, hdr->pkt_seq);
               }

               package_offset += current_package_len;
       }

       return packages_queued;
}

static uint16_t mctp_i3c_read_smq(void *mctp_p, uint8_t *buf, uint32_t len,
                                 mctp_ext_params *extra_data)
{
       CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
       CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);
       CHECK_NULL_ARG_WITH_RETURN(extra_data, MCTP_ERROR);

       mctp *mctp_inst = (mctp *)mctp_p;
       uint16_t rt_size = 0;
       uint8_t *raw_data;
       uint8_t dynamic_addr;
       I3C_MSG i3c_msg;
       uint16_t packages_queued;
       int ret = 0;

       /* Read from ring buffer */
       rt_size = mctp_i3c_smq_ringbuf_read(buf, len, extra_data);

       /* If no data is available in the ring buffer, read from I3C SMQ */
       if (rt_size == 0) {
               i3c_msg.bus = mctp_inst->medium_conf.i3c_conf.bus;
               ret = i3c_smq_read(&i3c_msg);

               if (ret <= 0) {
                       /* No data available */
                       return 0;
               } else {
                       raw_data = &i3c_msg.data[0];
                       i3c_msg.rx_len = ret;

                       dynamic_addr = 0;
                       if (MCTP_I3C_PEC_ENABLE) {
                               ret = i3c_target_get_dynamic_address(&i3c_msg, &dynamic_addr);
                               if (ret != 0) {
                                       LOG_ERR("Failed to get dynamic address for I3C bus: %x",
                                               i3c_msg.bus);
                                       return 0;
                               }
                       }

                       /* Queue MCTP packages */
                       packages_queued = queue_multiple_packages(
                               raw_data, i3c_msg.rx_len, mctp_inst->max_msg_size, dynamic_addr);
                       if (packages_queued > 1) {
                               LOG_INF("Queued %d MCTP packages from single I3C SMQ transfer",
                                       packages_queued);
                       }

                       /* Read from ring buffer after processing I3C SMQ data */
                       rt_size = mctp_i3c_smq_ringbuf_read(buf, len, extra_data);
               }
       }

       LOG_HEXDUMP_DBG(buf, rt_size, "mctp_i3c_read_smq msg dump");

       return rt_size;
}
#else
static uint16_t mctp_i3c_read_smq(void *mctp_p, uint8_t *buf, uint32_t len,
				  mctp_ext_params *extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(extra_data, MCTP_ERROR);

	int ret = 0;
	I3C_MSG i3c_msg;
	mctp *mctp_inst = (mctp *)mctp_p;
	i3c_msg.bus = mctp_inst->medium_conf.i3c_conf.bus;
	ret = i3c_smq_read(&i3c_msg);

	/** mctp rx keep polling, return length 0 directly if no data or invalid data **/
	if (ret <= 0) {
		return 0;
	}

	i3c_msg.rx_len = ret;
	LOG_HEXDUMP_DBG(&i3c_msg.data[0], i3c_msg.rx_len, "mctp_i3c_read_smq msg dump");

	if (MCTP_I3C_PEC_ENABLE) {
		uint8_t pec = 0x0, dynamic_addr = 0x0;

		ret = i3c_target_get_dynamic_address(&i3c_msg, &dynamic_addr);
		if (ret != 0) {
			LOG_ERR("Failed to get dynamic address for I3C bus: %x", i3c_msg.bus);
			return MCTP_ERROR;
		}
		/** pec byte use 7-degree polynomial with 0 init value and false reverse **/
		dynamic_addr = dynamic_addr << 1;
		pec = crc8(&dynamic_addr, 1, 0x07, 0x00, false);
		pec = crc8(&i3c_msg.data[0], i3c_msg.rx_len - 1, 0x07, pec, false);
		if (pec != i3c_msg.data[i3c_msg.rx_len - 1]) {
			LOG_ERR("mctp i3c pec error: crc8 should be 0x%02x, but got 0x%02x", pec,
				i3c_msg.data[i3c_msg.rx_len - 1]);
			return 0;
		}

		/** Remove pec byte if it is valid **/
		i3c_msg.rx_len--;
	}

	extra_data->type = MCTP_MEDIUM_TYPE_TARGET_I3C;
	memcpy(buf, &i3c_msg.data[0], i3c_msg.rx_len);
	return i3c_msg.rx_len;
}
#endif

static uint16_t mctp_i3c_write_smq(void *mctp_p, uint8_t *buf, uint32_t len,
				   mctp_ext_params extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);

	if (extra_data.type != MCTP_MEDIUM_TYPE_TARGET_I3C) {
		LOG_ERR("mctp medium type incorrect");
		return MCTP_ERROR;
	}

	int ret;
	I3C_MSG i3c_msg;
	mctp *mctp_instance = (mctp *)mctp_p;
	i3c_msg.bus = mctp_instance->medium_conf.i3c_conf.bus;
	/** mctp package **/
	if (len < I3C_MAX_DATA_SIZE) {
		memcpy(&i3c_msg.data[0], buf, len);
	} else {
		LOG_ERR("Write failed because I3C data length exceed %d bytes", I3C_MAX_DATA_SIZE);
		return MCTP_ERROR;
	}

	/** +1 pec; default no pec **/
	if (MCTP_I3C_PEC_ENABLE) {
		uint8_t pec = 0x0, dynamic_addr = 0x0;

		ret = i3c_target_get_dynamic_address(&i3c_msg, &dynamic_addr);
		if (ret != 0) {
			LOG_ERR("Failed to get dynamic address for I3C bus: %x", i3c_msg.bus);
			return MCTP_ERROR;
		}
		i3c_msg.tx_len = len + 1;
		/** pec byte use 7-degree polynomial with 0 init value and false reverse **/
		dynamic_addr = dynamic_addr << 1 | 1;
		pec = crc8(&dynamic_addr, 1, 0x07, 0x00, false);
		pec = crc8(&i3c_msg.data[0], len, 0x07, pec, false);
		i3c_msg.data[len] = pec;
	} else {
		i3c_msg.tx_len = len;
	}

	LOG_HEXDUMP_DBG(&i3c_msg.data[0], i3c_msg.tx_len, "mctp_i3c_write_smq msg dump");

	ret = i3c_smq_write(&i3c_msg);
	if (ret < 0) {
		LOG_ERR("mctp_i3c_write_smq write failed, %d", ret);
		return MCTP_ERROR;
	}
	return MCTP_SUCCESS;
}

uint8_t mctp_i3c_controller_init(mctp *mctp_instance, mctp_medium_conf medium_conf)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_instance, MCTP_ERROR);

	mctp_instance->medium_conf = medium_conf;
	mctp_instance->read_data = mctp_i3c_read;
	mctp_instance->write_data = mctp_i3c_write;

	// i3c master initial
	LOG_INF("Bus= 0x%x, Addr = 0x%x", medium_conf.i3c_conf.bus, medium_conf.i3c_conf.addr);
	I3C_MSG i3c_msg = { 0 };
	i3c_msg.bus = medium_conf.i3c_conf.bus;
	i3c_msg.target_addr = medium_conf.i3c_conf.addr;

	i3c_attach(&i3c_msg);

	// i3c ibi mqueue initial
	i3c_controller_ibi_init(&i3c_msg);

	return MCTP_SUCCESS;
}

uint8_t mctp_i3c_target_init(mctp *mctp_instance, mctp_medium_conf medium_conf)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_instance, MCTP_ERROR);

	mctp_instance->medium_conf = medium_conf;
	mctp_instance->read_data = mctp_i3c_read_smq;
	mctp_instance->write_data = mctp_i3c_write_smq;

	return MCTP_SUCCESS;
}

uint8_t mctp_i3c_deinit(mctp *mctp_instance)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_instance, MCTP_ERROR);

	mctp_instance->read_data = NULL;
	mctp_instance->write_data = NULL;
	memset(&mctp_instance->medium_conf, 0, sizeof(mctp_instance->medium_conf));
	return MCTP_SUCCESS;
}

#endif // ENABLE_MCTP_I3C
