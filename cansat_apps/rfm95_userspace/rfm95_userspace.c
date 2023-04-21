/**
 * @file rfm95.c
 * @author sCanSati Team 2020-2021, ITIS E. Fermi, Modena
 * @date 23 Mar 2021
 * @brief RFM95 (RFM9_) library for ESP32 - ESP-IDF framework
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "rfm95_userspace.h"
#include "nuttx/rf/rfm95.h"

/*
 * Register definitions
 */
#define REG_FIFO                       (0x00)
#define REG_OP_MODE                    (0x01)
#define REG_FRF_MSB                    (0x06)
#define REG_FRF_MID                    (0x07)
#define REG_FRF_LSB                    (0x08)
#define REG_PA_CONFIG                  (0x09)
#define REG_LNA                        (0x0c)
#define REG_FIFO_ADDR_PTR              (0x0d)
#define REG_FIFO_TX_BASE_ADDR          (0x0e)
#define REG_FIFO_RX_BASE_ADDR          (0x0f)
#define REG_FIFO_RX_CURRENT_ADDR       (0x10)
#define REG_IRQ_FLAGS                  (0x12)
#define REG_RX_NB_BYTES                (0x13)
#define REG_PKT_SNR_VALUE              (0x19)
#define REG_PKT_RSSI_VALUE             (0x1a)
#define REG_MODEM_CONFIG_1             (0x1d)
#define REG_MODEM_CONFIG_2             (0x1e)
#define REG_PREAMBLE_MSB               (0x20)
#define REG_PREAMBLE_LSB               (0x21)
#define REG_PAYLOAD_LENGTH             (0x22)
#define REG_MODEM_CONFIG_3             (0x26)
#define REG_RSSI_WIDEBAND              (0x2c)
#define REG_DETECTION_OPTIMIZE         (0x31)
#define REG_DETECTION_THRESHOLD        (0x37)
#define REG_SYNC_WORD                  (0x39)
#define REG_DIO_MAPPING_1              (0x40)
#define REG_VERSION                    (0x42)

/*
 * Transceiver modes
 */
#define MODE_LONG_RANGE_MODE           (0x80)
#define MODE_SLEEP                     (0x00)
#define MODE_STDBY                     (0x01)
#define MODE_TX                        (0x03)
#define MODE_RX_CONTINUOUS             (0x05)
#define MODE_RX_SINGLE                 (0x06)

/*
 * PA configuration
 */
#define PA_BOOST                       (0x80)

/*
 * IRQ masks
 */
#define IRQ_TX_DONE_MASK               (0x08)
#define IRQ_PAYLOAD_CRC_ERROR_MASK     (0x20)
#define IRQ_RX_DONE_MASK               (0x40)

#define PA_OUTPUT_RFO_PIN              (0)
#define PA_OUTPUT_PA_BOOST_PIN         (1)

#define TIMEOUT_RESET                  (100)

static int __implicit;
static long __frequency;

static int radio_fd; // < file descriptor of /dev/radio0, opened in rfm95_init()

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
void rfm95_write_reg(int reg, int val) {
   uint8_t out[2] = { 0x80 | reg, val };
   uint8_t in[2];

   write(radio_fd, out, 2);
   read(radio_fd, in, 2);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
int rfm95_read_reg(int reg) {
   uint8_t out[2] = { reg, 0xff };
   uint8_t in[2];

   write(radio_fd, out, 2);
   read(radio_fd, in, 2);
   return in[1];
}

void rfm95_reset() {
   ioctl(radio_fd, RFM95_IOCTL_RESET);
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void rfm95_explicit_header_mode(void) {
   __implicit = 0;
   rfm95_write_reg(REG_MODEM_CONFIG_1, rfm95_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void rfm95_implicit_header_mode(int size) {
   __implicit = 1;
   rfm95_write_reg(REG_MODEM_CONFIG_1, rfm95_read_reg(REG_MODEM_CONFIG_1) | 0x01);
   rfm95_write_reg(REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void rfm95_idle(void) {
   rfm95_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void rfm95_sleep(void) { 
   rfm95_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void rfm95_receive(void) {
   rfm95_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void rfm95_set_tx_power(int level) {
   // RF9x module uses PA_BOOST pin
   if (level < 2) level = 2;
   else if (level > 17) level = 17;
   rfm95_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void rfm95_set_frequency(long frequency) {
   __frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   rfm95_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   rfm95_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   rfm95_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void rfm95_set_spreading_factor(int sf) {
   if (sf < 6) sf = 6;
   else if (sf > 12) sf = 12;

   if (sf == 6) {
      rfm95_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
      rfm95_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
   } else {
      rfm95_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
      rfm95_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
   }

   rfm95_write_reg(REG_MODEM_CONFIG_2, (rfm95_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000)
 */
void rfm95_set_bandwidth(long sbw) {
   int bw;

   if (sbw <= 7.8E3) bw = 0;
   else if (sbw <= 10.4E3) bw = 1;
   else if (sbw <= 15.6E3) bw = 2;
   else if (sbw <= 20.8E3) bw = 3;
   else if (sbw <= 31.25E3) bw = 4;
   else if (sbw <= 41.7E3) bw = 5;
   else if (sbw <= 62.5E3) bw = 6;
   else if (sbw <= 125E3) bw = 7;
   else if (sbw <= 250E3) bw = 8;
   else bw = 9;
   rfm95_write_reg(REG_MODEM_CONFIG_1, (rfm95_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

/**
 * Set coding rate 
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */ 
void rfm95_set_coding_rate(int denominator) {
   if (denominator < 5) denominator = 5;
   else if (denominator > 8) denominator = 8;

   int cr = denominator - 4;
   rfm95_write_reg(REG_MODEM_CONFIG_1, (rfm95_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void rfm95_set_preamble_length(long length) {
   rfm95_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   rfm95_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void rfm95_set_sync_word(int sw) {
   rfm95_write_reg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
void rfm95_enable_crc(void) {
   rfm95_write_reg(REG_MODEM_CONFIG_2, rfm95_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void rfm95_disable_crc(void) {
   rfm95_write_reg(REG_MODEM_CONFIG_2, rfm95_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

/**
 * Perform hardware initialization.
 */
int rfm95_init(const char *dev_path) {
   radio_fd = open(dev_path, O_RDWR);
   assert(radio_fd > 0);

   /*
    * Perform hardware reset.
    */
   rfm95_reset();

   /*
    * Check version.
    */
   uint8_t version;
   uint8_t i = 0;
   while(i++ < TIMEOUT_RESET) {
      version = rfm95_read_reg(REG_VERSION);
      if(version == 0x12) break;
      up_mdelay(2);
   }
   DEBUGASSERT(i <= TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1

   /*
    * Default configuration.
    */
   rfm95_sleep();
   rfm95_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
   rfm95_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
   rfm95_write_reg(REG_LNA, rfm95_read_reg(REG_LNA) | 0x03);
   rfm95_write_reg(REG_MODEM_CONFIG_3, 0x04);
   rfm95_set_tx_power(17);

   rfm95_idle();
   return 1;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void rfm95_send_packet(const uint8_t *buf, int size) {
   /*
    * Transfer data to radio.
    */
   rfm95_idle();
   rfm95_write_reg(REG_FIFO_ADDR_PTR, 0);

   for(int i=0; i<size; i++) 
      rfm95_write_reg(REG_FIFO, *buf++);
   
   rfm95_write_reg(REG_PAYLOAD_LENGTH, size);
   
   /*
    * Start transmission and wait for conclusion.
    */
   rfm95_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
   while((rfm95_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
      vTaskDelay(2);

   rfm95_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int rfm95_receive_packet(uint8_t *buf, int size) {
   int len = 0;

   /*
    * Check interrupts.
    */
   int irq = rfm95_read_reg(REG_IRQ_FLAGS);
   rfm95_write_reg(REG_IRQ_FLAGS, irq);
   if((irq & IRQ_RX_DONE_MASK) == 0) return 0;
   if(irq & IRQ_PAYLOAD_CRC_ERROR_MASK) return 0;

   /*
    * Find packet size.
    */
   if (__implicit) len = rfm95_read_reg(REG_PAYLOAD_LENGTH);
   else len = rfm95_read_reg(REG_RX_NB_BYTES);

   /*
    * Transfer data from radio.
    */
   rfm95_idle();   
   rfm95_write_reg(REG_FIFO_ADDR_PTR, rfm95_read_reg(REG_FIFO_RX_CURRENT_ADDR));
   if(len > size) len = size;
   for(int i=0; i<len; i++) 
      *buf++ = rfm95_read_reg(REG_FIFO);

   return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int rfm95_received(void) {
   if(rfm95_read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) return 1;
   return 0;
}

/**
 * Return last packet's RSSI.
 */
int rfm95_packet_rssi(void) {
   return (rfm95_read_reg(REG_PKT_RSSI_VALUE) - (__frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float rfm95_packet_snr(void) {
   return ((int8_t)rfm95_read_reg(REG_PKT_SNR_VALUE)) * 0.25;
}

/**
 * Shutdown hardware.
 */
void rfm95_close(void) {
   rfm95_sleep();
   close(radio_fd);
}
