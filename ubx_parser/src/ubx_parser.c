/*
 * UBX Protocol Parser - Implementation
 *
 * Based on u-blox ubxlib (Apache 2.0 License)
 * Copyright 2019-2024 u-blox
 *
 * Simplified and modernized for C23 by Nantenaina RYCHEN
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ubx_parser.h"
#include "ubx_error.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/** Validate UBX message structure sizes at compile time */
_Static_assert(UBX_OVERHEAD_LEN == 8, "UBX overhead must be 8 bytes");
_Static_assert(UBX_HEADER_LEN == 6, "UBX header must be 6 bytes");

/* ----------------------------------------------------------------
 * PRIVATE FUNCTIONS
 * -------------------------------------------------------------- */

/**
 * @brief Update Fletcher-8 checksum (UBX CRC algorithm)
 * @param[in,out] crc_a Checksum A accumulator
 * @param[in,out] crc_b Checksum B accumulator
 * @param[in] byte Byte to add to checksum
 */
static inline void ubx_update_checksum(uint8_t *crc_a, uint8_t *crc_b, uint8_t byte)
{
	*crc_a += byte;
	*crc_b += *crc_a;
}

/**
 * @brief Calculate complete Fletcher-8 checksum for buffer
 * @param[in] data Buffer to checksum
 * @param[in] len Buffer length
 * @param[out] crc_a Checksum A result
 * @param[out] crc_b Checksum B result
 */
static void ubx_calculate_checksum(const uint8_t *data, size_t len, uint8_t *crc_a, uint8_t *crc_b)
{
	*crc_a = 0;
	*crc_b = 0;

	for (size_t i = 0; i < len; i++) {
		ubx_update_checksum(crc_a, crc_b, data[i]);
	}
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: Endianness utilities
 * -------------------------------------------------------------- */

bool ubx_is_little_endian(void)
{
	const uint32_t test = 1;
	return *(const uint8_t*)&test == 1;
}

uint16_t ubx_decode_u16_le(const uint8_t *data)
{
	return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

uint32_t ubx_decode_u32_le(const uint8_t *data)
{
	return (uint32_t)data[0] |
	((uint32_t)data[1] << 8) |
	((uint32_t)data[2] << 16) |
	((uint32_t)data[3] << 24);
}

uint64_t ubx_decode_u64_le(const uint8_t *data)
{
	return (uint64_t)data[0] |
	((uint64_t)data[1] << 8) |
	((uint64_t)data[2] << 16) |
	((uint64_t)data[3] << 24) |
	((uint64_t)data[4] << 32) |
	((uint64_t)data[5] << 40) |
	((uint64_t)data[6] << 48) |
	((uint64_t)data[7] << 56);
}

uint16_t ubx_encode_u16_le(uint16_t value)
{
	if (ubx_is_little_endian()) {
		return value;
	}

	return ((value & 0xFF00) >> 8) | ((value & 0x00FF) << 8);
}

uint32_t ubx_encode_u32_le(uint32_t value)
{
	if (ubx_is_little_endian()) {
		return value;
	}

	return ((value & 0xFF000000) >> 24) |
	((value & 0x00FF0000) >> 8) |
	((value & 0x0000FF00) << 8) |
	((value & 0x000000FF) << 24);
}

uint64_t ubx_encode_u64_le(uint64_t value)
{
	if (ubx_is_little_endian()) {
		return value;
	}

	return ((value & 0xFF00000000000000ULL) >> 56) |
	((value & 0x00FF000000000000ULL) >> 40) |
	((value & 0x0000FF0000000000ULL) >> 24) |
	((value & 0x000000FF00000000ULL) >> 8) |
	((value & 0x00000000FF000000ULL) << 8) |
	((value & 0x0000000000FF0000ULL) << 24) |
	((value & 0x000000000000FF00ULL) << 40) |
	((value & 0x00000000000000FFULL) << 56);
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: Message encoding/decoding
 * -------------------------------------------------------------- */

int32_t ubx_encode_message(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len, uint8_t *buffer)
{
	if (buffer == nullptr) {
		return UBX_ERROR_INVALID_PARAM;
	}

	if (payload_len > 0 && payload == nullptr) {
		return UBX_ERROR_INVALID_PARAM;
	}

	uint8_t *write_ptr = buffer;

	// Write header
	*write_ptr++ = UBX_SYNC1_BYTE;
	*write_ptr++ = UBX_SYNC2_BYTE;
	*write_ptr++ = msg_class;
	*write_ptr++ = msg_id;
	*write_ptr++ = (uint8_t)(payload_len & 0xFF);
	*write_ptr++ = (uint8_t)(payload_len >> 8);

	// Write payload if present
	if (payload != nullptr && payload_len > 0) {
		memcpy(write_ptr, payload, payload_len);
		write_ptr += payload_len;
	}

	// Calculate and write checksum (class + id + length + payload)
	uint8_t crc_a, crc_b;
	ubx_calculate_checksum(buffer + 2, 4 + payload_len, &crc_a, &crc_b);

	*write_ptr++ = crc_a;
	*write_ptr++ = crc_b;

	return (int32_t)(write_ptr - buffer);
}

ubx_result_t ubx_decode_message(const uint8_t *buffer, size_t buffer_len, ubx_message_t *msg, uint8_t *payload_buffer, size_t payload_buffer_len, size_t *bytes_consumed)
{
	if (buffer == nullptr || msg == nullptr) {
		return UBX_ERROR_INVALID_PARAM;
	}

	if (buffer_len < UBX_OVERHEAD_LEN) {
		return UBX_ERROR_PARTIAL;
	}

	// Search for sync bytes
	const uint8_t *sync_pos = nullptr;
	for (size_t i = 0; i <= buffer_len - UBX_OVERHEAD_LEN; i++) {
		if (buffer[i] == UBX_SYNC1_BYTE && buffer[i + 1] == UBX_SYNC2_BYTE) {
			sync_pos = &buffer[i];
			break;
		}
	}

	if (sync_pos == nullptr) {
		if (bytes_consumed) {
			*bytes_consumed = buffer_len;
		}
		return UBX_ERROR_NOT_FOUND;
	}

	const uint8_t *read_ptr = sync_pos;
	size_t remaining = buffer_len - (sync_pos - buffer);

	if (remaining < UBX_HEADER_LEN) {
		if (bytes_consumed) {
			*bytes_consumed = sync_pos - buffer;
		}
		return UBX_ERROR_PARTIAL;
	}

	// Parse header
	read_ptr += 2; // Skip sync bytes
	uint8_t msg_class = *read_ptr++;
	uint8_t msg_id = *read_ptr++;
	uint16_t payload_len = ubx_decode_u16_le(read_ptr);
	read_ptr += 2;

	// Check if complete message is available
	size_t total_msg_len = UBX_OVERHEAD_LEN + payload_len;
	if (remaining < total_msg_len) {
		if (bytes_consumed) {
			*bytes_consumed = sync_pos - buffer;
		}
		return UBX_ERROR_PARTIAL;
	}

	// Verify checksum
	uint8_t expected_crc_a, expected_crc_b;
	ubx_calculate_checksum(sync_pos + 2, 4 + payload_len, &expected_crc_a, &expected_crc_b);

	uint8_t received_crc_a = sync_pos[UBX_HEADER_LEN + payload_len];
	uint8_t received_crc_b = sync_pos[UBX_HEADER_LEN + payload_len + 1];

	if (expected_crc_a != received_crc_a || expected_crc_b != received_crc_b) {
		if (bytes_consumed) {
			*bytes_consumed = (sync_pos - buffer) + 1; // Skip this sync byte
		}
		return UBX_ERROR_BAD_DATA;
	}

	// Fill message structure
	msg->msg_class = msg_class;
	msg->msg_id = msg_id;
	msg->payload_len = payload_len;
	msg->payload = (payload_len > 0) ? read_ptr : nullptr;

	// Copy payload if buffer provided
	if (payload_buffer != nullptr && payload_len > 0) {
		size_t copy_len = (payload_len > payload_buffer_len) ? payload_buffer_len : payload_len;
		memcpy(payload_buffer, read_ptr, copy_len);
		msg->payload = payload_buffer; // Update pointer to copied data
	}

	if (bytes_consumed) {
		*bytes_consumed = (sync_pos - buffer) + total_msg_len;
	}

	return UBX_OK;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: Streaming parser
 * -------------------------------------------------------------- */

void ubx_parser_init(ubx_parser_t *parser)
{
	if (parser == nullptr) {
		return;
	}

	parser->state = UBX_PARSE_SYNC1;
	parser->payload_len = 0;
	parser->bytes_received = 0;
	parser->msg_class = 0;
	parser->msg_id = 0;
	parser->crc_a = 0;
	parser->crc_b = 0;
}

ubx_result_t ubx_parser_feed_byte(ubx_parser_t *parser, uint8_t byte, uint8_t *payload_buffer, size_t payload_buffer_len)
{
	if (parser == nullptr) {
		return UBX_ERROR_INVALID_PARAM;
	}

	switch (parser->state) {
		case UBX_PARSE_SYNC1:
			if (byte == UBX_SYNC1_BYTE) {
				parser->state = UBX_PARSE_SYNC2;
			}
			break;

		case UBX_PARSE_SYNC2:
			if (byte == UBX_SYNC2_BYTE) {
				parser->state = UBX_PARSE_CLASS;
				parser->crc_a = 0;
				parser->crc_b = 0;
			} else {
				parser->state = (byte == UBX_SYNC1_BYTE) ? UBX_PARSE_SYNC2 : UBX_PARSE_SYNC1;
			}
			break;

		case UBX_PARSE_CLASS:
			parser->msg_class = byte;
			ubx_update_checksum(&parser->crc_a, &parser->crc_b, byte);
			parser->state = UBX_PARSE_ID;
			break;

		case UBX_PARSE_ID:
			parser->msg_id = byte;
			ubx_update_checksum(&parser->crc_a, &parser->crc_b, byte);
			parser->state = UBX_PARSE_LEN_LOW;
			break;

		case UBX_PARSE_LEN_LOW:
			parser->payload_len = byte;
			ubx_update_checksum(&parser->crc_a, &parser->crc_b, byte);
			parser->state = UBX_PARSE_LEN_HIGH;
			break;

		case UBX_PARSE_LEN_HIGH:
			parser->payload_len |= ((uint16_t)byte) << 8;
			ubx_update_checksum(&parser->crc_a, &parser->crc_b, byte);
			parser->bytes_received = 0;
			parser->state = (parser->payload_len > 0) ? UBX_PARSE_PAYLOAD : UBX_PARSE_CRC_A;
			break;

		case UBX_PARSE_PAYLOAD:
			// Store payload byte if buffer available
			if (payload_buffer != nullptr && parser->bytes_received < payload_buffer_len) {
				payload_buffer[parser->bytes_received] = byte;
			}

			ubx_update_checksum(&parser->crc_a, &parser->crc_b, byte);
			parser->bytes_received++;

			if (parser->bytes_received >= parser->payload_len) {
				parser->state = UBX_PARSE_CRC_A;
			}
			break;

		case UBX_PARSE_CRC_A:
			if (byte == (parser->crc_a & 0xFF)) {
				parser->state = UBX_PARSE_CRC_B;
			} else {
				parser->state = UBX_PARSE_SYNC1; // Checksum error, restart
				return UBX_ERROR_BAD_DATA;
			}
			break;

		case UBX_PARSE_CRC_B:
			if (byte == (parser->crc_b & 0xFF)) {
				parser->state = UBX_PARSE_COMPLETE;
				return UBX_OK; // Message complete!
			} else {
				parser->state = UBX_PARSE_SYNC1; // Checksum error, restart
				return UBX_ERROR_BAD_DATA;
			}
			break;

		case UBX_PARSE_COMPLETE:
			// Should not happen - user should call reset after getting UBX_OK
			parser->state = UBX_PARSE_SYNC1;
			// Fall through to handle this byte as start of new message
			return ubx_parser_feed_byte(parser, byte, payload_buffer, payload_buffer_len);
	}

	return UBX_ERROR_PARTIAL; // More data needed
}

void ubx_parser_get_message(const ubx_parser_t *parser, ubx_message_t *msg)
{
	if (parser == nullptr || msg == nullptr) {
		return;
	}

	msg->msg_class = parser->msg_class;
	msg->msg_id = parser->msg_id;
	msg->payload_len = parser->payload_len;
	msg->payload = nullptr; // User must manage payload buffer separately
}

void ubx_parser_reset(ubx_parser_t *parser)
{
	if (parser != nullptr) {
		parser->state = UBX_PARSE_SYNC1;
	}
}
