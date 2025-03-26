
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "hal/usb_serial_jtag_ll.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include <SPI.h>
#include "w25q.h"

#define LED_PIN       8

uint8_t buf[256];     // Read data buffer
uint16_t n;           // Number of data read

// Declare the flash object of W25Q class globally
W25Q flash;

#define GPIO_MOSI	    6 
#define GPIO_MISO	    5 
#define GPIO_SCLK	    4 
#define GPIO_CS1      9  // PSRAM64H (8MB)
#define GPIO_CS2     10  // W25Q128 (16MB)

#define SPI_HOST_ID	  SPI2_HOST
#define SPI_FREQ	    40000000 // 80MHz

#define CMD_WRITE	    0x02
#define CMD_READ	    0x03
#define CMD_FAST_READ	0x0b
#define CMD_RESET_EN	0x66
#define CMD_RESET	    0x99
#define CMD_READ_ID	  0x9f

static spi_device_handle_t handle;

static esp_err_t psram_send_cmd(spi_device_handle_t h, const uint8_t cmd)
{
	spi_transaction_ext_t t = { };
	t.base.flags = SPI_TRANS_VARIABLE_ADDR;
	t.base.cmd = cmd;
	t.base.length = 0;
        t.command_bits = 8U;
        t.address_bits = 0;

	return spi_device_polling_transmit(h, (spi_transaction_t*)&t);
}

static esp_err_t psram_read_id(spi_device_handle_t h, uint8_t *rxdata)
{
	spi_transaction_t t = { };
	t.cmd = CMD_READ_ID;
	t.addr = 0;
	t.rx_buffer = rxdata;
	t.length = 6 * 8;
	return spi_device_polling_transmit(h, &t);
}

int psram_init(void)
{
	esp_err_t ret;
	uint8_t id[6];

	gpio_reset_pin((gpio_num_t)GPIO_CS1);
	gpio_set_direction((gpio_num_t)GPIO_CS1, GPIO_MODE_OUTPUT);
	gpio_set_level((gpio_num_t)GPIO_CS1, 1);

	spi_bus_config_t spi_bus_config = {
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = GPIO_MISO,
		.sclk_io_num = GPIO_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
		.flags = 0,
	};

	printf("SPI_HOST_ID = %d\r\n", (gpio_num_t)SPI_HOST_ID);
 	ret = spi_bus_initialize(SPI_HOST_ID, &spi_bus_config, SPI_DMA_CH_AUTO);
	printf("spi_bus_initialize = %d\r\n", ret);
	if (ret != ESP_OK)
		return -1;

	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
	devcfg.clock_speed_hz = SPI_FREQ;
	devcfg.spics_io_num = -1;
	devcfg.queue_size = 1;
	devcfg.command_bits = 8;
	devcfg.address_bits = 24;

	ret = spi_bus_add_device(SPI_HOST_ID, &devcfg, &handle);
	printf("spi_bus_add_device = %d\r\n", ret);
	if (ret != ESP_OK)
		return -1;

	gpio_set_level((gpio_num_t)GPIO_CS1, 1);
	usleep(200);

	gpio_set_level((gpio_num_t)GPIO_CS1, 0);
	psram_send_cmd(handle, CMD_RESET_EN);
	psram_send_cmd(handle, CMD_RESET);
	gpio_set_level((gpio_num_t)GPIO_CS1, 1);
	usleep(200);

	gpio_set_level((gpio_num_t)GPIO_CS1, 0);
	ret = psram_read_id(handle, id);
	gpio_set_level((gpio_num_t)GPIO_CS1, 1);
	if (ret != ESP_OK)
		return -1;
  // ESP-IDF PSRAM ID: 0d5d52d26fd0(80MHZ),ARDUINO ESP32 PSRAM ID: 0d5d50b1c027(40MHZ)
	printf("PSRAM ID: %02x%02x%02x%02x%02x%02x\r\n", id[0], id[1], id[2], id[3], id[4], id[5]);
	return 0;
}

int psram_read(uint32_t addr, void *buf, int len)
{
	esp_err_t ret;
	spi_transaction_ext_t t = { };

	t.base.cmd = CMD_FAST_READ;
	t.base.addr = addr;
	t.base.rx_buffer = buf;
	t.base.length = len * 8;
	t.base.flags = SPI_TRANS_VARIABLE_DUMMY;
	t.dummy_bits = 8;

	gpio_set_level((gpio_num_t)GPIO_CS1, 0);
	ret = spi_device_polling_transmit(handle, (spi_transaction_t*)&t);
	gpio_set_level((gpio_num_t)GPIO_CS1, 1);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "psram_read failed %lx %d\r\n", addr, len);
		return -1;
	}

	return len;
}

int psram_write(uint32_t addr, void *buf, int len)
{
	esp_err_t ret;
	spi_transaction_t t = {};

	t.cmd = CMD_WRITE;
	t.addr = addr;
	t.tx_buffer = buf;
	t.length = len * 8;

	gpio_set_level((gpio_num_t)GPIO_CS1, 0);
	ret = spi_device_polling_transmit(handle, &t);
	gpio_set_level((gpio_num_t)GPIO_CS1, 1);

	if (ret != ESP_OK) {
		// Replacing printf with ESP_LOGE
		ESP_LOGE(TAG, "psram_write failed 0x%lx %d\r\n", addr, len);
		return -1;
	}

	return len;
}

#if 1
#define TEST_ADDR 0x00001000  // 테스트할 PSRAM 주소
#define TEST_DATA "HelloPSRAM"
#define BUF_SIZE  16

void test_psram() {
  char write_data[] = TEST_DATA;
  char read_data[BUF_SIZE] = {0};

  printf("Writing to PSRAM...\r\n");
  if (psram_write(TEST_ADDR, write_data, strlen(write_data) + 1) < 0) {
     printf("Write failed!\r\n");
    return;
  }

  printf("Reading from PSRAM...\r\n");
  if (psram_read(TEST_ADDR, read_data, strlen(write_data) + 1) < 0) {
    printf("Read failed!\r\n");
    return;
  }

  printf("Read data: %s\r\n", read_data);
    
  if (strcmp(write_data, read_data) == 0) {
    printf("PSRAM Test Passed!\r\n");
  } else {
    printf("PSRAM Test Failed! Data mismatch.\r\n");
  }
}
#endif


void setup()
{
  // Initialize UART with baudrate 115200
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Disable the task watchdog timer
  esp_timer_deinit();
             
  usleep(5000000);  // Sleep for 5seconds
    
  // Print banner
  Serial.println(F("EPS32C3: Linux on Arduino Platform"));
   
  if (psram_init() == 0) {
    Serial.println("PSRAM Initialized Successfully!");
  } else {
    Serial.println("PSRAM Initialization Failed!");
  }

  test_psram();

  // Initialize the W25Q flash memory
  if (!flash.disk_initialize()) {
    Serial.println("W25Q128 Initialization Failed!");
    while (1);  // Infinite loop in case of initialization failure
  }
  Serial.println("W25Q128 Initialized Successfully!");

  // Get and print the JEDEC ID to confirm chip identification
  uint32_t jedec_id = flash.get_JEDEC_ID();
  Serial.print("JEDEC ID: ");
  Serial.println(jedec_id, HEX);

  flash.getUID(w25q_data.uuid);  // Get UID into the w25q_data.uuid array

  // Print the UID as a string of hexadecimal values
  Serial.print("UID: ");
  for (int i = 0; i < 8; i++) {
    if (i > 0) {  // Add ':' between each byte
      Serial.print(":");
    }
    if (w25q_data.uuid[i] < 0x10) {
      Serial.print("0");  // Print leading zero for values < 0x10
    }
    Serial.print(w25q_data.uuid[i], HEX);
  }
  Serial.println();

  flash.Write_Byte(20, 'C');  // Write 'C' to address 20
  flash.Write_Byte(32, 'T');  // Write 'T' to address 32
  flash.Write_Byte(36, 'M');  // Write 'M' to address 36
  
  // Wait to ensure the writes complete
  delay(10);  // Adjust the delay as needed for your flash write time
  
  char readChar = flash.Read_Byte(20);
  Serial.print("Data at address 20: ");
  Serial.println(readChar);  // Should print 'C'
  readChar = flash.Read_Byte(32);
  Serial.print("Data at address 32: ");
  Serial.println(readChar);  // Should print 'T'
  readChar = flash.Read_Byte(36);
  Serial.print("Data at address 36: ");
  Serial.println(readChar);  // Should print 'M'
}

void loop()
{
 
}
