#include <assert.h>
#include <stdio.h>
#include "../control.h"

static void tick(CarControl *c, int32_t *l, int32_t *r, float *yaw, uint32_t *now)
{
    ObstacleState o = { .sonar_result = SONAR_NO_ECHO, .measured_ms = *now };
    float lp, rp;
    *now += 10;
    control_step(c, 0, 0, *l, *r, *yaw, 0, .01f, *now, &o, &lp, &rp);
}

int main(void)
{
    CarControl c;
    int32_t l = 0, r = 0;
    float yaw = 0;
    uint32_t now = 0;
    control_init(&c);
    assert(c.counts_per_meter == 3250 && c.target == 20 && c.mode == MODE_STRAIGHT);
    control_keys(&c, 0, 0, 0, 1, ++now);
    control_keys(&c, 0, 0, 0, 0, ++now);
    assert(c.mode == MODE_SQUARE);
    control_start(&c, yaw, l, r, now);
    for (unsigned edge = 0; edge < 4; ++edge) {
        l += 3250; r += 3250;
        tick(&c, &l, &r, &yaw, &now);
        tick(&c, &l, &r, &yaw, &now);
        assert(c.phase == ROUTE_BRAKE);
        for (unsigned i = 0; i < 30; ++i) tick(&c, &l, &r, &yaw, &now);
        assert(c.phase == ROUTE_TURN_OUT);
        yaw = wrap_angle(-(float)(edge + 1) * 90.0f);
        for (unsigned i = 0; i < 25; ++i) tick(&c, &l, &r, &yaw, &now);
        if (edge < 3) assert(c.phase == ROUTE_SQUARE_FORWARD);
    }
    assert(!c.running && c.stop_reason == STOP_DONE);

    control_init(&c);
    control_keys(&c, 0, 1, 0, 0, 0);
    control_keys(&c, 0, 0, 0, 0, 1);
    assert(c.setting == SETTING_DISTANCE);
    control_keys(&c, 0, 0, 0, 1, 2);
    control_keys(&c, 0, 0, 0, 0, 3);
    assert(c.setting == SETTING_SPEED);
    control_keys(&c, 0, 0, 0, 1, 4);
    control_keys(&c, 0, 0, 0, 0, 5);
    assert(c.setting == SETTING_COUNTS);
    control_keys(&c, 1, 0, 0, 0, 6);
    assert(c.counts_per_meter == 3260);
    control_keys(&c, 0, 1, 0, 0, 7);
    assert(c.setting == SETTING_GPS);
    control_keys(&c, 0, 0, 0, 0, 8);
    control_keys(&c, 0, 1, 0, 0, 9);
    assert(c.setting == SETTING_NONE);

    c.distance_m = 2;
    c.target = 8;
    c.counts_per_meter = 2000;
    control_start(&c, 0, 0, 0, 10);
    ObstacleState clear = { .sonar_result = SONAR_NO_ECHO };
    float lp, rp;
    control_step(&c, 0, 0, 100, 100, 0, 0, .01f, 20, &clear, &lp, &rp);
    assert(c.distance_m == 2 && c.target == 8 && c.counts_per_meter == 2000);
    assert(c.x > 0.049f && c.x < 0.051f && lp < 10 && rp < 10);
    puts("mode and square self-check: PASS");
    return 0;
}
