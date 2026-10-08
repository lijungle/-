#ifndef ESP_LINK_H
#define ESP_LINK_H
#include <stdint.h>
#define ESP_BUFFER_SIZE 512U
enum { ESP_STOP, ESP_FORWARD, ESP_BACKWARD, ESP_LEFT, ESP_RIGHT };
typedef struct { uint8_t cmd, speed; uint32_t duration_ms; } ESPCommand;
typedef struct {
    uint8_t rx[ESP_BUFFER_SIZE], tx[ESP_BUFFER_SIZE];
    volatile uint16_t rx_head, rx_tail, tx_head, tx_tail;
    volatile uint8_t rx_overflow, tx_overflow;
    char line[160];
    uint16_t length;
    uint8_t dropping;
} ESPLink;
void esp_init(ESPLink *link);
void esp_rx_byte(ESPLink *link, uint8_t byte);
int esp_poll(ESPLink *link, ESPCommand *command);
int esp_tx_byte(ESPLink *link, uint8_t *byte);
int esp_position(ESPLink *link, float lat, float lng, float heading, float speed);
#endif
