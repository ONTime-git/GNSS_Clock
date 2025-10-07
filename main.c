/* USER CODE BEGIN Header */
/**
 * *****************************************************************************
 * @file           : main.c
 * @brief          : GNSS u-blox DAN-F10N - Non-blocking UBX Parser with FIFO
 * @author         : Nantenaina RYCHEN
 * @standard       : C23 strict
 ******************************************************************************
 * Features:
 * - UBX protocol only (NMEA disabled)
 * - All constellations & frequencies enabled
 * - 1Hz navigation rate
 * - DMA circular buffers on both UARTs
 * - Thread-safe FIFO queues for TX/RX (no data loss)
 * - Atomic thread-safe navigation data
 * - Interactive VCP commands
 * - LED toggle on valid GNSS fix
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ubx_parser.h"
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include <stdarg.h>

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VCP_UART                hlpuart1
#define GNSS_UART               hlpuart2

#define VCP_RX_BUFFER_SIZE      256
#define GNSS_RX_BUFFER_SIZE     512
#define GNSS_PAYLOAD_MAX        256

// FIFO queue sizes
#define VCP_TX_QUEUE_SIZE       24
#define GNSS_TX_QUEUE_SIZE      24
#define VCP_TX_MSG_MAX_LEN      768
#define GNSS_TX_MSG_MAX_LEN     (GNSS_PAYLOAD_MAX + 8)

// UBX Message Classes and IDs
#define UBX_CLASS_CFG           0x06
#define UBX_CLASS_NAV           0x01
#define UBX_CLASS_MON           0x0A

#define UBX_CFG_VALSET          0x8A
#define UBX_CFG_RATE            0x01
#define UBX_NAV_PVT             0x07
#define UBX_MON_VER             0x04

/* USER CODE END PD */

/* Private types -------------------------------------------------------------*/
/* USER CODE BEGIN PT */

// Generic FIFO queue structure (thread-safe with atomics)
typedef struct {
	uint8_t *buffer;
	_Atomic(size_t) head;
	_Atomic(size_t) tail;
	size_t capacity;
	size_t msg_size;
	_Atomic(size_t) count;
} fifo_queue_t;

// TX message structures
typedef struct {
	uint8_t data[VCP_TX_MSG_MAX_LEN];
	uint16_t length;
	bool valid;
} vcp_tx_msg_t;

typedef struct {
	uint8_t data[GNSS_TX_MSG_MAX_LEN];
	uint16_t length;
	bool valid;
} gnss_tx_msg_t;

// Thread-safe atomic navigation data (UBX-NAV-PVT)
typedef struct {
	_Atomic(uint32_t) iTOW;
	_Atomic(int32_t) lon;
	_Atomic(int32_t) lat;
	_Atomic(int32_t) height;
	_Atomic(int32_t) hMSL;
	_Atomic(uint32_t) hAcc;
	_Atomic(uint32_t) vAcc;
	atomic_bool valid;
	_Atomic(uint8_t) fixType;
	_Atomic(uint8_t) numSV;
	_Atomic(uint16_t) year;
	_Atomic(uint8_t) month;
	_Atomic(uint8_t) day;
	_Atomic(uint8_t) hour;
	_Atomic(uint8_t) minute;
	_Atomic(uint8_t) second;
	_Atomic(int32_t) velN;
	_Atomic(int32_t) velE;
	_Atomic(int32_t) velD;
	_Atomic(uint32_t) gSpeed;
	_Atomic(int32_t) headMot;
	_Atomic(uint32_t) sAcc;
	_Atomic(uint32_t) headAcc;
	_Atomic(uint16_t) pDOP;
} nav_data_t;

/* USER CODE END PT */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef hlpuart2;
DMA_HandleTypeDef hdma_lpuart1_rx;
DMA_HandleTypeDef hdma_lpuart1_tx;
DMA_HandleTypeDef hdma_lpuart2_rx;
DMA_HandleTypeDef hdma_lpuart2_tx;

// DMA circular buffers
static uint8_t vcp_rx_buffer[VCP_RX_BUFFER_SIZE];
static uint8_t gnss_rx_buffer[GNSS_RX_BUFFER_SIZE];
static volatile size_t vcp_last_pos = 0;
static volatile size_t gnss_last_pos = 0;

// TX FIFO queues and buffers
static vcp_tx_msg_t vcp_tx_queue_buffer[VCP_TX_QUEUE_SIZE];
static gnss_tx_msg_t gnss_tx_queue_buffer[GNSS_TX_QUEUE_SIZE];
static fifo_queue_t vcp_tx_queue;
static fifo_queue_t gnss_tx_queue;

// TX state flags
static _Atomic(bool) vcp_tx_busy = false;
static _Atomic(bool) gnss_tx_busy = false;

// Current TX messages being sent
static vcp_tx_msg_t current_vcp_tx;
static gnss_tx_msg_t current_gnss_tx;

// UBX parser state
static ubx_parser_t gnss_parser;
static uint8_t gnss_payload_buffer[GNSS_PAYLOAD_MAX];

// Thread-safe navigation data
static nav_data_t nav_data = {0};

// Command buffer
static char cmd_buffer[128];
static size_t cmd_idx = 0;

// Statistics
static _Atomic(uint32_t) vcp_tx_dropped = 0;
static _Atomic(uint32_t) gnss_tx_dropped = 0;
static _Atomic(uint32_t) ubx_messages_received = 0;
static _Atomic(uint32_t) ubx_checksum_errors = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_LPUART2_UART_Init(void);

/* USER CODE BEGIN PFP */
// FIFO queue functions
static void fifo_init(fifo_queue_t *q, void *buffer, size_t capacity, size_t msg_size);
static bool fifo_push(fifo_queue_t *q, const void *data, size_t len);
static bool fifo_pop(fifo_queue_t *q, void *data, size_t *len);
static size_t fifo_count(const fifo_queue_t *q);
static bool fifo_is_empty(const fifo_queue_t *q);

// UART TX functions
static void vcp_tx_queue_push(const uint8_t *data, uint16_t len);
static void gnss_tx_queue_push(const uint8_t *data, uint16_t len);
static void vcp_tx_process(void);
static void gnss_tx_process(void);

// GNSS functions
static void gnss_configure(void);
static void gnss_process_rx(void);
static void handle_ubx_message(const ubx_message_t *msg);
static void process_nav_pvt(const uint8_t *payload, uint16_t len);
static int32_t gnss_send_ubx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
static void gnss_enable_all_signals(void);

// VCP functions
static void vcp_process_rx(void);
static void process_command(const char *cmd);
static void send_message(const char *fmt, ...);

// Utility functions
[[maybe_unused]] static void safe_printf_int(char *buf, size_t bufsize, const char *prefix, long value, const char *suffix);
[[maybe_unused]] static void safe_printf_frac(char *buf, size_t bufsize, long int_part, long frac_part, int frac_digits);

/* USER CODE END PFP */

/**
 * @brief  The application entry point.
 */
int main(void)
{
	/* MCU Configuration */
	HAL_Init();
	SystemClock_Config();

	/* Initialize peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_LPUART1_UART_Init();
	MX_LPUART2_UART_Init();

	/* USER CODE BEGIN 2 */

	// Initialize FIFO queues
	fifo_init(&vcp_tx_queue, vcp_tx_queue_buffer, VCP_TX_QUEUE_SIZE, sizeof(vcp_tx_msg_t));
	fifo_init(&gnss_tx_queue, gnss_tx_queue_buffer, GNSS_TX_QUEUE_SIZE, sizeof(gnss_tx_msg_t));

	// Start DMA reception on both UARTs
	HAL_UART_Receive_DMA(&VCP_UART, vcp_rx_buffer, VCP_RX_BUFFER_SIZE);
	HAL_UART_Receive_DMA(&GNSS_UART, gnss_rx_buffer, GNSS_RX_BUFFER_SIZE);

	// Initialize UBX parser
	ubx_parser_init(&gnss_parser);

	// Welcome message
	send_message("\r\n=== GNSS u-blox DAN-F10N ===\r\n");
	send_message("Firmware: C23 Non-blocking UBX Parser with FIFO\r\n");
	send_message("Type 'help' for commands\r\n\r\n");

	// Configure GNSS module
	HAL_Delay(500);
	gnss_configure();

	/* USER CODE END 2 */

	/* Infinite loop */
	uint32_t last_tick = HAL_GetTick();

	while (1)
	{
		// Process GNSS RX data (non-blocking)
		gnss_process_rx();

		// Process VCP RX commands (non-blocking)
		vcp_process_rx();

		// Process TX queues (non-blocking)
		vcp_tx_process();
		gnss_tx_process();

		// Periodic status LED update (1Hz)
		if ((HAL_GetTick() - last_tick) >= 1000) {
			last_tick = HAL_GetTick();

			if (atomic_load(&nav_data.valid) && atomic_load(&nav_data.fixType) >= 2) {
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			}
		}
	}
}

/* USER CODE BEGIN 4 */

/* ============================================================================
 * Utility Functions
 * ========================================================================= */

[[maybe_unused]] static void safe_printf_int(char *buf, size_t bufsize, const char *prefix, long value, const char *suffix)
{
	snprintf(buf, bufsize, "%s%ld%s", prefix, value, suffix);
}

[[maybe_unused]] static void safe_printf_frac(char *buf, size_t bufsize, long int_part, long frac_part, int frac_digits)
{
	char fmt[32];
	snprintf(fmt, sizeof(fmt), "%%ld.%%0%dld", frac_digits);
	snprintf(buf, bufsize, fmt, int_part, frac_part);
}

/* ============================================================================
 * FIFO Queue Implementation (Thread-safe with atomics)
 * ========================================================================= */

static void fifo_init(fifo_queue_t *q, void *buffer, size_t capacity, size_t msg_size)
{
	q->buffer = (uint8_t*)buffer;
	atomic_store(&q->head, 0);
	atomic_store(&q->tail, 0);
	q->capacity = capacity;
	q->msg_size = msg_size;
	atomic_store(&q->count, 0);
}

static bool fifo_push(fifo_queue_t *q, const void *data, size_t len)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	size_t count = atomic_load(&q->count);
	if (count >= q->capacity) {
		__set_PRIMASK(primask);
		return false;
	}

	size_t head = atomic_load(&q->head);
	uint8_t *slot = q->buffer + (head * q->msg_size);

	if (len <= q->msg_size) {
		memcpy(slot, data, len);
	} else {
		__set_PRIMASK(primask);
		return false;
	}

	atomic_store(&q->head, (head + 1) % q->capacity);
	size_t new_count = count + 1;
	atomic_store(&q->count, new_count);

	__set_PRIMASK(primask);
	return true;
}

static bool fifo_pop(fifo_queue_t *q, void *data, size_t *len)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	size_t count = atomic_load(&q->count);
	if (count == 0) {
		__set_PRIMASK(primask);
		return false;
	}

	size_t tail = atomic_load(&q->tail);
	uint8_t *slot = q->buffer + (tail * q->msg_size);

	memcpy(data, slot, q->msg_size);
	if (len) {
		*len = q->msg_size;
	}

	atomic_store(&q->tail, (tail + 1) % q->capacity);
	size_t new_count = count - 1;
	atomic_store(&q->count, new_count);

	__set_PRIMASK(primask);
	return true;
}

static size_t fifo_count(const fifo_queue_t *q)
{
	return atomic_load(&q->count);
}

static bool fifo_is_empty(const fifo_queue_t *q)
{
	return atomic_load(&q->count) == 0;
}

/* ============================================================================
 * UART TX Queue Management
 * ========================================================================= */

static void vcp_tx_queue_push(const uint8_t *data, uint16_t len)
{
	if (len > VCP_TX_MSG_MAX_LEN) {
		len = VCP_TX_MSG_MAX_LEN;
	}

	vcp_tx_msg_t msg = {0};
	memcpy(msg.data, data, len);
	msg.length = len;
	msg.valid = true;

	if (!fifo_push(&vcp_tx_queue, &msg, sizeof(msg))) {
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		uint32_t dropped = atomic_load(&vcp_tx_dropped);
		atomic_store(&vcp_tx_dropped, dropped + 1);
		__set_PRIMASK(primask);
	}
}

static void gnss_tx_queue_push(const uint8_t *data, uint16_t len)
{
	if (len > GNSS_TX_MSG_MAX_LEN) {
		len = GNSS_TX_MSG_MAX_LEN;
	}

	gnss_tx_msg_t msg = {0};
	memcpy(msg.data, data, len);
	msg.length = len;
	msg.valid = true;

	if (!fifo_push(&gnss_tx_queue, &msg, sizeof(msg))) {
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		uint32_t dropped = atomic_load(&gnss_tx_dropped);
		atomic_store(&gnss_tx_dropped, dropped + 1);
		__set_PRIMASK(primask);
	}
}

static void vcp_tx_process(void)
{
	if (atomic_load(&vcp_tx_busy)) {
		return;
	}

	if (!fifo_is_empty(&vcp_tx_queue)) {
		size_t len;
		if (fifo_pop(&vcp_tx_queue, &current_vcp_tx, &len)) {
			if (current_vcp_tx.valid && current_vcp_tx.length > 0) {
				atomic_store(&vcp_tx_busy, true);
				HAL_UART_Transmit_DMA(&VCP_UART, current_vcp_tx.data, current_vcp_tx.length);
			}
		}
	}
}

static void gnss_tx_process(void)
{
	if (atomic_load(&gnss_tx_busy)) {
		return;
	}

	if (!fifo_is_empty(&gnss_tx_queue)) {
		size_t len;
		if (fifo_pop(&gnss_tx_queue, &current_gnss_tx, &len)) {
			if (current_gnss_tx.valid && current_gnss_tx.length > 0) {
				atomic_store(&gnss_tx_busy, true);
				HAL_UART_Transmit_DMA(&GNSS_UART, current_gnss_tx.data, current_gnss_tx.length);
			}
		}
	}
}

/* ============================================================================
 * HAL UART Callbacks (called from IRQ context)
 * ========================================================================= */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == LPUART1) {
		atomic_store(&vcp_tx_busy, false);
	} else if (huart->Instance == LPUART2) {
		atomic_store(&gnss_tx_busy, false);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == LPUART1) {
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
		atomic_store(&vcp_tx_busy, false);
		HAL_UART_Receive_DMA(&VCP_UART, vcp_rx_buffer, VCP_RX_BUFFER_SIZE);
	} else if (huart->Instance == LPUART2) {
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
		atomic_store(&gnss_tx_busy, false);
		HAL_UART_Receive_DMA(&GNSS_UART, gnss_rx_buffer, GNSS_RX_BUFFER_SIZE);
	}
}

/* ============================================================================
 * VCP Functions
 * ========================================================================= */

static void send_message(const char *fmt, ...)
{
	static char tx_buffer[VCP_TX_MSG_MAX_LEN];
	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(tx_buffer, sizeof(tx_buffer), fmt, args);
	va_end(args);

	if (len > 0 && len < (int)sizeof(tx_buffer)) {
		vcp_tx_queue_push((uint8_t*)tx_buffer, (uint16_t)len);
	}
}

static void vcp_process_rx(void)
{
	size_t current_pos = VCP_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(VCP_UART.hdmarx);

	while (vcp_last_pos != current_pos) {
		char c = (char)vcp_rx_buffer[vcp_last_pos];
		vcp_last_pos = (vcp_last_pos + 1) % VCP_RX_BUFFER_SIZE;

		// Echo character
		uint8_t echo = (uint8_t)c;
		vcp_tx_queue_push(&echo, 1);

		if (c == '\r' || c == '\n') {
			if (cmd_idx > 0) {
				cmd_buffer[cmd_idx] = '\0';
				send_message("\r\n");
				process_command(cmd_buffer);
				cmd_idx = 0;
			}
		} else if (c == 0x08 || c == 0x7F) {
			if (cmd_idx > 0) cmd_idx--;
		} else if (cmd_idx < sizeof(cmd_buffer) - 1) {
			cmd_buffer[cmd_idx++] = c;
		}
	}
}

static void process_command(const char *cmd)
{
	if (strcmp(cmd, "help") == 0) {
		send_message("Available commands:\r\n");
		send_message("  help   - Show this help\r\n");
		send_message("  status - Show system status\r\n");
		send_message("  fix    - Show GNSS fix data\r\n");
		send_message("  stats  - Show statistics\r\n\r\n");
	}
	else if (strcmp(cmd, "status") == 0) {
		send_message("System Status:\r\n");
		send_message("  Clock: 56 MHz (HSI+PLL)\r\n");
		send_message("  VCP: LPUART1 @ 115200 baud\r\n");
		send_message("  GNSS: LPUART2 @ 38400 baud\r\n");
		send_message("  Constellations: GPS, Galileo, BeiDou, QZSS\r\n");
		send_message("  Nav rate: 1 Hz\r\n");

		unsigned vcp_q = (unsigned)fifo_count(&vcp_tx_queue);
		unsigned gnss_q = (unsigned)fifo_count(&gnss_tx_queue);
		send_message("  VCP TX queue: %u / %u\r\n", vcp_q, VCP_TX_QUEUE_SIZE);
		send_message("  GNSS TX queue: %u / %u\r\n\r\n", gnss_q, GNSS_TX_QUEUE_SIZE);
	}
	else if (strcmp(cmd, "stats") == 0) {
		uint32_t rx_cnt = atomic_load(&ubx_messages_received);
		uint32_t err_cnt = atomic_load(&ubx_checksum_errors);
		uint32_t vcp_drop = atomic_load(&vcp_tx_dropped);
		uint32_t gnss_drop = atomic_load(&gnss_tx_dropped);

		send_message("Statistics:\r\n");
		send_message("  UBX messages RX: %lu\r\n", (unsigned long)rx_cnt);
		send_message("  UBX checksum errors: %lu\r\n", (unsigned long)err_cnt);
		send_message("  VCP TX dropped: %lu\r\n", (unsigned long)vcp_drop);
		send_message("  GNSS TX dropped: %lu\r\n\r\n", (unsigned long)gnss_drop);
	}
	else if (strcmp(cmd, "fix") == 0) {
		bool valid = atomic_load(&nav_data.valid);
		uint8_t fixType = atomic_load(&nav_data.fixType);

		send_message("\r\n=== GNSS Fix Data ===\r\n\r\n");

		if (!valid || fixType < 2) {
			send_message("No valid GNSS fix\r\n");
			send_message("  Fix type: %u (need >= 2)\r\n", fixType);
			send_message("  Valid: %u\r\n\r\n", valid ? 1 : 0);
			return;
		}

		// Read all data atomically
		uint16_t year = atomic_load(&nav_data.year);
		uint8_t month = atomic_load(&nav_data.month);
		uint8_t day = atomic_load(&nav_data.day);
		uint8_t hour = atomic_load(&nav_data.hour);
		uint8_t minute = atomic_load(&nav_data.minute);
		uint8_t second = atomic_load(&nav_data.second);
		uint32_t iTOW = atomic_load(&nav_data.iTOW);

		int32_t lat = atomic_load(&nav_data.lat);
		int32_t lon = atomic_load(&nav_data.lon);
		int32_t alt = atomic_load(&nav_data.hMSL);
		int32_t height = atomic_load(&nav_data.height);

		uint32_t hAcc = atomic_load(&nav_data.hAcc);
		uint32_t vAcc = atomic_load(&nav_data.vAcc);
		uint16_t pDOP = atomic_load(&nav_data.pDOP);

		int32_t velN = atomic_load(&nav_data.velN);
		int32_t velE = atomic_load(&nav_data.velE);
		int32_t velD = atomic_load(&nav_data.velD);
		uint32_t gSpeed = atomic_load(&nav_data.gSpeed);
		int32_t headMot = atomic_load(&nav_data.headMot);
		uint32_t sAcc = atomic_load(&nav_data.sAcc);

		uint8_t numSV = atomic_load(&nav_data.numSV);

		// Date & Time section
		send_message("Date/Time (UTC):\r\n");
		if (year == 0 && month == 0 && day == 0) {
			send_message("  Not available\r\n");
		} else {
			send_message("  %02u/%02u/%04u %02u:%02u:%02u\r\n",
						 day, month, year, hour, minute, second);
		}
		send_message("  iTOW: %lu ms\r\n\r\n", (unsigned long)iTOW);

		// Fix info section
		const char* fixTypeStr = (fixType == 3) ? "3D Fix" : "2D Fix";
		send_message("Fix Information:\r\n");
		send_message("  Type: %s\r\n", fixTypeStr);
		send_message("  Satellites: %u\r\n", numSV);

		int pdop_int = pDOP / 100;
		int pdop_frac = pDOP % 100;
		send_message("  PDOP: %d.%02d\r\n\r\n", pdop_int, pdop_frac);

		// Position section - split into multiple sends to avoid overflow
		long lat_deg = lat / 10000000L;
		long lat_frac = (lat < 0 ? -lat : lat) % 10000000L;
		long lon_deg = lon / 10000000L;
		long lon_frac = (lon < 0 ? -lon : lon) % 10000000L;

		send_message("Position:\r\n");
		send_message("  Lat: %c%ld.%07ld deg\r\n",
					 lat >= 0 ? '+' : '-',
			   lat_deg < 0 ? -lat_deg : lat_deg,
			   lat_frac);
		send_message("  Lon: %c%ld.%07ld deg\r\n",
					 lon >= 0 ? '+' : '-',
			   lon_deg < 0 ? -lon_deg : lon_deg,
			   lon_frac);

		long alt_m = alt / 1000L;
		long alt_mm = (alt < 0 ? -alt : alt) % 1000L;
		send_message("  Alt (MSL): %c%ld.%03ld m\r\n",
					 alt >= 0 ? '+' : '-',
			   alt_m < 0 ? -alt_m : alt_m,
			   alt_mm);

		long height_m = height / 1000L;
		long height_mm = (height < 0 ? -height : height) % 1000L;
		send_message("  Alt (Ellip): %c%ld.%03ld m\r\n\r\n",
					 height >= 0 ? '+' : '-',
			   height_m < 0 ? -height_m : height_m,
			   height_mm);

		// Accuracy section
		long hAcc_m = hAcc / 1000L;
		long hAcc_mm = hAcc % 1000L;
		long vAcc_m = vAcc / 1000L;
		long vAcc_mm = vAcc % 1000L;

		send_message("Accuracy:\r\n");
		send_message("  Horiz: %ld.%03ld m\r\n", hAcc_m, hAcc_mm);
		send_message("  Vert:  %ld.%03ld m\r\n\r\n", vAcc_m, vAcc_mm);

		// Velocity section - split into multiple sends
		long gSpeed_ms = gSpeed / 1000L;
		long gSpeed_frac_ms = gSpeed % 1000L;
		long gSpeed_kmh = (gSpeed * 36L) / 10000L;
		long gSpeed_frac_kmh = ((gSpeed * 36L) / 100L) % 100L;

		send_message("Velocity:\r\n");
		send_message("  Speed: %ld.%02ld km/h\r\n", gSpeed_kmh, gSpeed_frac_kmh);
		send_message("         (%ld.%03ld m/s)\r\n", gSpeed_ms, gSpeed_frac_ms);

		long head_deg = headMot / 100000L;
		long head_frac = (headMot < 0 ? -headMot : headMot) % 100000L / 1000L;
		send_message("  Heading: %ld.%02ld deg\r\n", head_deg, head_frac);

		long sAcc_ms = sAcc / 1000L;
		long sAcc_frac = sAcc % 1000L;
		send_message("  Spd acc: %ld.%03ld m/s\r\n", sAcc_ms, sAcc_frac);

		long velN_ms = velN / 1000L;
		long velN_frac = (velN < 0 ? -velN : velN) % 1000L;
		send_message("  North: %c%ld.%03ld m/s\r\n",
					 velN >= 0 ? '+' : '-',
			   velN_ms < 0 ? -velN_ms : velN_ms,
			   velN_frac);

		long velE_ms = velE / 1000L;
		long velE_frac = (velE < 0 ? -velE : velE) % 1000L;
		send_message("  East:  %c%ld.%03ld m/s\r\n",
					 velE >= 0 ? '+' : '-',
			   velE_ms < 0 ? -velE_ms : velE_ms,
			   velE_frac);

		long velD_ms = velD / 1000L;
		long velD_frac = (velD < 0 ? -velD : velD) % 1000L;
		send_message("  Down:  %c%ld.%03ld m/s\r\n\r\n",
					 velD >= 0 ? '+' : '-',
			   velD_ms < 0 ? -velD_ms : velD_ms,
			   velD_frac);
	}
	else {
		send_message("Unknown command: %s\r\n", cmd);
		send_message("Type 'help'\r\n\r\n");
	}
}

/* ============================================================================
 * GNSS Functions
 * ========================================================================= */

static void gnss_configure(void)
{
	send_message("Configuring GNSS...\r\n");

	gnss_send_ubx(UBX_CLASS_MON, UBX_MON_VER, NULL, 0);
	HAL_Delay(100);

	gnss_enable_all_signals();
	HAL_Delay(100);

	uint8_t rate_cfg[] = {
		0xE8, 0x03, // measRate = 1000ms
		0x01, 0x00, // navRate = 1
		0x01, 0x00  // timeRef = GPS
	};
	gnss_send_ubx(UBX_CLASS_CFG, UBX_CFG_RATE, rate_cfg, sizeof(rate_cfg));
	HAL_Delay(100);

	uint8_t valset_pvt[] = {
		0x00,       // version
		0x01,       // layers (RAM)
		0x00, 0x00, // reserved
		0x07, 0x00, 0x91, 0x20, // CFG_MSGOUT_UBX_NAV_PVT_UART1
		0x01        // output rate = 1
	};
	gnss_send_ubx(UBX_CLASS_CFG, UBX_CFG_VALSET, valset_pvt, sizeof(valset_pvt));

	send_message("GNSS configured: All constellations, 1Hz\r\n\r\n");
}

static void gnss_enable_all_signals(void)
{
	uint8_t valset_signals[] = {
		0x00,       // version
		0x01,       // layers (RAM)
		0x00, 0x00, // reserved

		// GPS
		0x1F, 0x00, 0x31, 0x10, 0x01,
		0x01, 0x00, 0x31, 0x10, 0x01,
		0x04, 0x00, 0x31, 0x10, 0x01,

		// Galileo
		0x21, 0x00, 0x31, 0x10, 0x01,
		0x07, 0x00, 0x31, 0x10, 0x01,
		0x09, 0x00, 0x31, 0x10, 0x01,

		// BeiDou
		0x22, 0x00, 0x31, 0x10, 0x01,
		0x0F, 0x00, 0x31, 0x10, 0x01,
		0x28, 0x00, 0x31, 0x10, 0x01,

		// QZSS
		0x24, 0x00, 0x31, 0x10, 0x01,
		0x12, 0x00, 0x31, 0x10, 0x01,
		0x17, 0x00, 0x31, 0x10, 0x01
	};

	gnss_send_ubx(UBX_CLASS_CFG, UBX_CFG_VALSET, valset_signals, sizeof(valset_signals));
}

static int32_t gnss_send_ubx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
	uint8_t tx_buffer[GNSS_TX_MSG_MAX_LEN];
	int32_t msg_len = ubx_encode_message(cls, id, payload, len, tx_buffer);

	if (msg_len > 0) {
		gnss_tx_queue_push(tx_buffer, (uint16_t)msg_len);
		return msg_len;
	}
	return msg_len;
}

static void gnss_process_rx(void)
{
	size_t current_pos = GNSS_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(GNSS_UART.hdmarx);

	while (gnss_last_pos != current_pos) {
		uint8_t byte = gnss_rx_buffer[gnss_last_pos];
		gnss_last_pos = (gnss_last_pos + 1) % GNSS_RX_BUFFER_SIZE;

		ubx_result_t result = ubx_parser_feed_byte(&gnss_parser, byte,
												   gnss_payload_buffer,
											 GNSS_PAYLOAD_MAX);

		if (result == UBX_OK) {
			ubx_message_t msg;
			ubx_parser_get_message(&gnss_parser, &msg);
			msg.payload = gnss_payload_buffer;

			uint32_t primask = __get_PRIMASK();
			__disable_irq();
			uint32_t rx_count = atomic_load(&ubx_messages_received);
			atomic_store(&ubx_messages_received, rx_count + 1);
			__set_PRIMASK(primask);

			handle_ubx_message(&msg);
			ubx_parser_reset(&gnss_parser);
		} else if (result == UBX_ERROR_BAD_DATA) {
			uint32_t primask = __get_PRIMASK();
			__disable_irq();
			uint32_t err_count = atomic_load(&ubx_checksum_errors);
			atomic_store(&ubx_checksum_errors, err_count + 1);
			__set_PRIMASK(primask);

			ubx_parser_reset(&gnss_parser);
		}
	}
}

static void handle_ubx_message(const ubx_message_t *msg)
{
	if (msg->msg_class == UBX_CLASS_NAV && msg->msg_id == UBX_NAV_PVT) {
		process_nav_pvt(msg->payload, msg->payload_len);
	}
}

static void process_nav_pvt(const uint8_t *payload, uint16_t len)
{
	if (len < 92) return;

	// Extract time fields (offsets 0-11 in UBX-NAV-PVT)
	uint32_t iTOW = ((uint32_t)payload[0]) |
	((uint32_t)payload[1] << 8) |
	((uint32_t)payload[2] << 16) |
	((uint32_t)payload[3] << 24);

	uint16_t year = ((uint16_t)payload[4]) | ((uint16_t)payload[5] << 8);
	uint8_t month = payload[6];
	uint8_t day = payload[7];
	uint8_t hour = payload[8];
	uint8_t minute = payload[9];
	uint8_t second = payload[10];
	uint8_t valid = payload[11];

	// Extract fix fields (offsets 20-23)
	uint8_t fixType = payload[20];
	[[maybe_unused]] uint8_t flags = payload[21];
	uint8_t numSV = payload[23];

	// Extract position fields (offsets 24-47)
	int32_t lon = ((int32_t)payload[24]) |
	((int32_t)payload[25] << 8) |
	((int32_t)payload[26] << 16) |
	((int32_t)payload[27] << 24);

	int32_t lat = ((int32_t)payload[28]) |
	((int32_t)payload[29] << 8) |
	((int32_t)payload[30] << 16) |
	((int32_t)payload[31] << 24);

	int32_t height = ((int32_t)payload[32]) |
	((int32_t)payload[33] << 8) |
	((int32_t)payload[34] << 16) |
	((int32_t)payload[35] << 24);

	int32_t hMSL = ((int32_t)payload[36]) |
	((int32_t)payload[37] << 8) |
	((int32_t)payload[38] << 16) |
	((int32_t)payload[39] << 24);

	uint32_t hAcc = ((uint32_t)payload[40]) |
	((uint32_t)payload[41] << 8) |
	((uint32_t)payload[42] << 16) |
	((uint32_t)payload[43] << 24);

	uint32_t vAcc = ((uint32_t)payload[44]) |
	((uint32_t)payload[45] << 8) |
	((uint32_t)payload[46] << 16) |
	((uint32_t)payload[47] << 24);

	// Extract velocity fields (offsets 48-75)
	int32_t velN = ((int32_t)payload[48]) |
	((int32_t)payload[49] << 8) |
	((int32_t)payload[50] << 16) |
	((int32_t)payload[51] << 24);

	int32_t velE = ((int32_t)payload[52]) |
	((int32_t)payload[53] << 8) |
	((int32_t)payload[54] << 16) |
	((int32_t)payload[55] << 24);

	int32_t velD = ((int32_t)payload[56]) |
	((int32_t)payload[57] << 8) |
	((int32_t)payload[58] << 16) |
	((int32_t)payload[59] << 24);

	uint32_t gSpeed = ((uint32_t)payload[60]) |
	((uint32_t)payload[61] << 8) |
	((uint32_t)payload[62] << 16) |
	((uint32_t)payload[63] << 24);

	int32_t headMot = ((int32_t)payload[64]) |
	((int32_t)payload[65] << 8) |
	((int32_t)payload[66] << 16) |
	((int32_t)payload[67] << 24);

	uint32_t sAcc = ((uint32_t)payload[68]) |
	((uint32_t)payload[69] << 8) |
	((uint32_t)payload[70] << 16) |
	((uint32_t)payload[71] << 24);

	uint32_t headAcc = ((uint32_t)payload[72]) |
	((uint32_t)payload[73] << 8) |
	((uint32_t)payload[74] << 16) |
	((uint32_t)payload[75] << 24);

	uint16_t pDOP = ((uint16_t)payload[76]) | ((uint16_t)payload[77] << 8);

	// Atomically update navigation data
	atomic_store(&nav_data.year, year);
	atomic_store(&nav_data.month, month);
	atomic_store(&nav_data.day, day);
	atomic_store(&nav_data.hour, hour);
	atomic_store(&nav_data.minute, minute);
	atomic_store(&nav_data.second, second);
	atomic_store(&nav_data.iTOW, iTOW);
	atomic_store(&nav_data.lon, lon);
	atomic_store(&nav_data.lat, lat);
	atomic_store(&nav_data.height, height);
	atomic_store(&nav_data.hMSL, hMSL);
	atomic_store(&nav_data.hAcc, hAcc);
	atomic_store(&nav_data.vAcc, vAcc);
	atomic_store(&nav_data.fixType, fixType);
	atomic_store(&nav_data.numSV, numSV);
	atomic_store(&nav_data.velN, velN);
	atomic_store(&nav_data.velE, velE);
	atomic_store(&nav_data.velD, velD);
	atomic_store(&nav_data.gSpeed, gSpeed);
	atomic_store(&nav_data.headMot, headMot);
	atomic_store(&nav_data.sAcc, sAcc);
	atomic_store(&nav_data.headAcc, headAcc);
	atomic_store(&nav_data.pDOP, pDOP);
	atomic_store(&nav_data.valid, (valid & 0x03) == 0x03);
}

/* USER CODE END 4 */

/* ============================================================================
 * System Configuration
 * ========================================================================= */

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 7;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_LPUART1_UART_Init(void)
{
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 115200;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_LPUART2_UART_Init(void)
{
	hlpuart2.Instance = LPUART2;
	hlpuart2.Init.BaudRate = 38400;
	hlpuart2.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart2.Init.StopBits = UART_STOPBITS_1;
	hlpuart2.Init.Parity = UART_PARITY_NONE;
	hlpuart2.Init.Mode = UART_MODE_TX_RX;
	hlpuart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	hlpuart2.FifoMode = UART_FIFOMODE_DISABLE;
	if (HAL_UART_Init(&hlpuart2) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_DMA_Init(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX_OVR_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX_OVR_IRQn);
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	// Button PC13
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// LED PA5
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
	__disable_irq();
	while (1) {
		// Blink LED rapidly to indicate error
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		for (volatile uint32_t i = 0; i < 100000; i++);
	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
	send_message("ASSERT FAILED: %s:%lu\r\n", file, line);
}
#endif
