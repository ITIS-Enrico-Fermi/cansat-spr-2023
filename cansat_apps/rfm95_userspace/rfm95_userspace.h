#pragma once

void rfm95_reset(void);
void rfm95_explicit_header_mode(void);
void rfm95_implicit_header_mode(int size);
void rfm95_idle(void);
void rfm95_sleep(void); 
void rfm95_receive(void);
void rfm95_set_tx_power(int level);
void rfm95_set_frequency(long frequency);
void rfm95_set_spreading_factor(int sf);
void rfm95_set_bandwidth(long sbw);
void rfm95_set_coding_rate(int denominator);
void rfm95_set_preamble_length(long length);
void rfm95_set_sync_word(int sw);
void rfm95_enable_crc(void);
void rfm95_disable_crc(void);
int rfm95_init(const char *dev_path;
void rfm95_send_packet(const uint8_t *buf, int size);
int rfm95_receive_packet(uint8_t *buf, int size);
int rfm95_received(void);
int rfm95_packet_rssi(void);
float rfm95_packet_snr(void);
void rfm95_close(void);
int rfm95_initialized(void);
void rfm95_dump_registers(void);
