#include <Arduino.h>

uint32_t lorawan_get_count();
void lorawan_send(uint8_t * data, uint8_t data_size, uint8_t port, bool confirmed);
size_t lorawan_response_len();
void lorawan_response(uint8_t * buffer, size_t len);
bool lorawan_setup();
void lorawan_join();
void lorawan_adr(bool enabled);
void lorawan_loop();

//void ConnectionStatues (bool Connected);
