#include <assert.h>
#include <stdio.h>
#include "../gps.h"

int main(void)
{
    GPSState gps;
    static const char gga[] = "$GNGGA,123519.00,2230.1234,N,11356.5678,E,1,08,0.90,10.0,M,0.0,M,,*44\r\n";
    static const char rmc[] = "$GNRMC,123519.00,A,2230.1234,N,11356.5678,E,0.1,0.0,010203,,,A*47\r\n";
    gps_init(&gps);
    for (const char *p = gga; *p; ++p) gps_isr_byte(&gps, (unsigned char)*p);
    gps_poll(&gps, 100);
    assert(gps.valid && gps.quality == 1 && gps.latitude > 22.5f && gps.longitude > 113.9f);
    assert(gps.hdop > 0.89f && gps.hdop < 0.91f);
    for (const char *p = rmc; *p; ++p) gps_isr_byte(&gps, (unsigned char)*p);
    gps_poll(&gps, 200);
    assert(gps.valid && gps.last_fix_ms == 200);
    puts("GPS parser self-check: PASS");
    return 0;
}
