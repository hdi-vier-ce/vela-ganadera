#include <Arduino.h>

uint32_t ttn_get_count();
void ttn_send(uint8_t * data, uint8_t data_size, uint8_t port, bool confirmed);
size_t ttn_response_len();
void ttn_response(uint8_t * buffer, size_t len);
bool ttn_setup();
void ttn_join();
void ttn_adr(bool enabled);
void ttn_loop();
