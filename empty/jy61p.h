#ifndef JY61P_H
#define JY61P_H
#include <stdint.h>
typedef struct { uint8_t buf[11], pos; float yaw, gyro_z; uint32_t last_ms; uint8_t valid; } JY61P;
void jy61p_init(JY61P *s);
int jy61p_feed(JY61P *s, uint8_t b, uint32_t now_ms);
int jy61p_timeout(const JY61P *s, uint32_t now_ms);
#endif
