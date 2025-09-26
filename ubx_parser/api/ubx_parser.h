/*
 * UBX Protocol Parser - Header
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

#ifndef UBX_PARSER_H
#define UBX_PARSER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "ubx_error.h"

/** @file
 * @brief UBX protocol message encoder/decoder for embedded systems
 */

#ifdef __cplusplus
extern "C" {
	#endif

	/* ----------------------------------------------------------------
	 * COMPILE-TIME MACROS
	 * -------------------------------------------------------------- */

	/** UBX protocol constants */
	#define UBX_SYNC1_BYTE        0xB5    ///< First sync byte
	#define UBX_SYNC2_BYTE        0x62    ///< Second sync byte
	#define UBX_HEADER_LEN        6       ///< Header length (sync + class + id + len)
	#define UBX_CRC_LEN           2       ///< CRC length
	#define UBX_OVERHEAD_LEN      (UBX_HEADER_LEN + UBX_CRC_LEN)  ///< Total overhead
	#define UBX_MAX_PAYLOAD_LEN   65535   ///< Maximum payload length

	/** Endianness conversion macros using C23 typeof */
	#define UBX_LE16(ptr) ubx_decode_u16_le((const uint8_t*)(ptr))
	#define UBX_LE32(ptr) ubx_decode_u32_le((const uint8_t*)(ptr))
	#define UBX_LE64(ptr) ubx_decode_u64_le((const uint8_t*)(ptr))

	/* ----------------------------------------------------------------
	 * TYPES
	 * -------------------------------------------------------------- */

	/** UBX message structure */
	typedef struct {
		uint8_t msg_class;           ///< Message class
		uint8_t msg_id;              ///< Message ID
		uint16_t payload_len;        ///< Payload length
		const uint8_t *payload;      ///< Pointer to payload data
	} ubx_message_t;

	/** UBX parser context for streaming decode */
	typedef struct {
		ubx_parse_state_t state;     ///< Current parser state
		uint16_t payload_len;        ///< Expected payload length
		uint16_t bytes_received;     ///< Bytes received so far
		uint8_t msg_class;           ///< Current message class
		uint8_t msg_id;              ///< Current message ID
		uint8_t crc_a;               ///< Running CRC A
		uint8_t crc_b;               ///< Running CRC B
	} ubx_parser_t;

	/* ----------------------------------------------------------------
	 * PUBLIC FUNCTIONS
	 * -------------------------------------------------------------- */

	/** @name Endianness utilities */
	/** @{ */

	/**
	 * @brief Check if the processor is little-endian
	 * @return true if little-endian, false if big-endian
	 */
	[[nodiscard]] bool ubx_is_little_endian(void);

	/**
	 * @brief Decode little-endian uint16_t from buffer
	 * @param[in] data Pointer to 2-byte buffer
	 * @return Decoded uint16_t value
	 */
	[[nodiscard]] uint16_t ubx_decode_u16_le(const uint8_t *data);

	/**
	 * @brief Decode little-endian uint32_t from buffer
	 * @param[in] data Pointer to 4-byte buffer
	 * @return Decoded uint32_t value
	 */
	[[nodiscard]] uint32_t ubx_decode_u32_le(const uint8_t *data);

	/**
	 * @brief Decode little-endian uint64_t from buffer
	 * @param[in] data Pointer to 8-byte buffer
	 * @return Decoded uint64_t value
	 */
	[[nodiscard]] uint64_t ubx_decode_u64_le(const uint8_t *data);

	/**
	 * @brief Encode uint16_t to little-endian format
	 * @param value Value to encode
	 * @return Little-endian encoded value
	 */
	[[nodiscard]] uint16_t ubx_encode_u16_le(uint16_t value);

	/**
	 * @brief Encode uint32_t to little-endian format
	 * @param value Value to encode
	 * @return Little-endian encoded value
	 */
	[[nodiscard]] uint32_t ubx_encode_u32_le(uint32_t value);

	/**
	 * @brief Encode uint64_t to little-endian format
	 * @param value Value to encode
	 * @return Little-endian encoded value
	 */
	[[nodiscard]] uint64_t ubx_encode_u64_le(uint64_t value);

	/** @} */

	/** @name Message encoding/decoding */
	/** @{ */

	/**
	 * @brief Encode a UBX message into a buffer
	 *
	 * @param[in] msg_class Message class (e.g., 0x01 for NAV)
	 * @param[in] msg_id Message ID (e.g., 0x07 for NAV-PVT)
	 * @param[in] payload Pointer to payload data (can be nullptr for empty payload)
	 * @param[in] payload_len Length of payload in bytes
	 * @param[out] buffer Output buffer (must be at least payload_len + UBX_OVERHEAD_LEN bytes)
	 * @return Number of bytes written to buffer, or negative error code
	 */
	[[nodiscard]] int32_t ubx_encode_message(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len, uint8_t *buffer);

	/**
	 * @brief Decode a complete UBX message from buffer
	 *
	 * This function searches for and decodes the first complete UBX message in the buffer.
	 *
	 * @param[in] buffer Input buffer containing UBX data
	 * @param[in] buffer_len Length of input buffer
	 * @param[out] msg Decoded message structure
	 * @param[out] payload_buffer Buffer to store decoded payload (can be nullptr to get size only)
	 * @param[in] payload_buffer_len Size of payload buffer
	 * @param[out] bytes_consumed Pointer to store number of bytes consumed from buffer (can be nullptr)
	 * @return UBX_OK on success, or error code. Returns payload length even if payload_buffer is nullptr
	 */
	[[nodiscard]] ubx_result_t ubx_decode_message(const uint8_t *buffer, size_t buffer_len, ubx_message_t *msg, uint8_t *payload_buffer, size_t payload_buffer_len, size_t *bytes_consumed);

	/** @} */

	/** @name Streaming parser */
	/** @{ */

	/**
	 * @brief Initialize UBX parser for streaming decode
	 * @param[out] parser Parser context to initialize
	 */
	void ubx_parser_init(ubx_parser_t *parser);

	/**
	 * @brief Parse a single byte in streaming mode
	 *
	 * @param[in,out] parser Parser context
	 * @param[in] byte Input byte to parse
	 * @param[out] payload_buffer Buffer to store payload bytes (can be nullptr)
	 * @param[in] payload_buffer_len Size of payload buffer
	 * @return UBX_OK if message complete, UBX_ERROR_PARTIAL if more data needed,
	 *         or other error code
	 */
	[[nodiscard]] ubx_result_t ubx_parser_feed_byte(ubx_parser_t *parser, uint8_t byte, uint8_t *payload_buffer, size_t payload_buffer_len);

	/**
	 * @brief Get the current message from parser (call after UBX_OK from feed_byte)
	 * @param[in] parser Parser context
	 * @param[out] msg Message structure to fill
	 */
	void ubx_parser_get_message(const ubx_parser_t *parser, ubx_message_t *msg);

	/**
	 * @brief Reset parser state (e.g., after error or timeout)
	 * @param[out] parser Parser context to reset
	 */
	void ubx_parser_reset(ubx_parser_t *parser);

	/** @} */

	#ifdef __cplusplus
}
#endif

#endif // UBX_PARSER_H
