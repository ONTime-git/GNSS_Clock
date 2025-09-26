/*
 * UBX Protocol Parser - Error Definitions
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

#ifndef UBX_ERROR_H
#define UBX_ERROR_H

#include <stdint.h>

/** @file
 * @brief UBX protocol error codes and result types
 */

#ifdef __cplusplus
extern "C" {
	#endif

	/* ----------------------------------------------------------------
	 * COMPILE-TIME MACROS
	 * -------------------------------------------------------------- */

	/** Base for negative error codes */
	#ifndef UBX_ERROR_BASE
	#define UBX_ERROR_BASE 0
	#endif

	/* ----------------------------------------------------------------
	 * TYPES
	 * -------------------------------------------------------------- */

	/** UBX operation result codes */
	typedef enum {
		UBX_OK = 0,										//< Operation successful
		UBX_ERROR_UNKNOWN = UBX_ERROR_BASE - 1,			//< Unknown error
		UBX_ERROR_INVALID_PARAM = UBX_ERROR_BASE - 2,	//< Invalid parameter
		UBX_ERROR_NO_MEMORY = UBX_ERROR_BASE - 3,		//< Out of memory
		UBX_ERROR_TIMEOUT = UBX_ERROR_BASE - 4,			//< Operation timed out
		UBX_ERROR_NOT_FOUND = UBX_ERROR_BASE - 5,		//< Item not found
		UBX_ERROR_BAD_DATA = UBX_ERROR_BASE - 6,		//< Corrupted data
		UBX_ERROR_TOO_BIG = UBX_ERROR_BASE - 7,			//< Data too large
		UBX_ERROR_PARTIAL = UBX_ERROR_BASE - 8,			//< Partial message received

		UBX_RESULT_FORCE_INT32 = 0x7FFFFFFF				//< Force enum to 32-bit
	} ubx_result_t;

	/** Parser state for streaming decode */
	typedef enum {
		UBX_PARSE_SYNC1 = 0,	//< Waiting for first sync byte (0xB5)
		UBX_PARSE_SYNC2,		//< Waiting for second sync byte (0x62)
		UBX_PARSE_CLASS,		//< Reading message class
		UBX_PARSE_ID,			//< Reading message ID
		UBX_PARSE_LEN_LOW,		//< Reading length low byte
		UBX_PARSE_LEN_HIGH,		//< Reading length high byte
		UBX_PARSE_PAYLOAD,		//< Reading payload data
		UBX_PARSE_CRC_A,		//< Reading first CRC byte
		UBX_PARSE_CRC_B,		//< Reading second CRC byte
		UBX_PARSE_COMPLETE		//< Message parsing complete
	} ubx_parse_state_t;

	#ifdef __cplusplus
}
#endif

#endif // UBX_ERROR_H
