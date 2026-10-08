/* 主机自检：验证真实协议缓冲区和真实控制路径。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../esp_link.h"
#include "../control.h"

/******************************************************************
 * 函 数 名 称：feed
 * 函 数 说 明：模拟串口中断逐字节收帧
 * 函 数 形 参：link - 通信对象；text - 数据
 * 函 数 返 回：无
 ******************************************************************/
static void feed(ESPLink *link, const char *text)
{
    while (*text) esp_rx_byte(link, (uint8_t)*text++);
}

/******************************************************************
 * 函 数 名 称：main
 * 函 数 说 明：覆盖分段接收、环形折返、格式拒绝、溢出及定时遥控
 * 函 数 形 参：无
 * 函 数 返 回：0 表示通过
 ******************************************************************/
int main(void)
{
    ESPLink link;
    ESPCommand command = {0};
    CarControl car;
    ObstacleState clear = {.sonar_result = SONAR_NO_ECHO};
    float lp, rp;
    esp_init(&link);
    feed(&link, "{\"type\":\"control\",\"cmd\":\"forward\",");
    assert(!esp_poll(&link, &command));
    feed(&link, "\"speed\":50,\"duration_ms\":500}\r\n");
    assert(esp_poll(&link, &command));
    assert(command.cmd == ESP_FORWARD && command.speed == 50 && command.duration_ms == 500);
    for (int i = 0; i < 30; ++i) {
        feed(&link, "{ \"cmd\": \"stop\", \"type\": \"control\" }\n");
        assert(esp_poll(&link, &command) && command.cmd == ESP_STOP);
    }
    const char *bad[] = {
        "{\"type\":\"control\",\"cmd\":\"forward\",\"speed\":101,\"duration_ms\":500}\n",
        "{\"type\":\"control\",\"cmd\":\"forward\",\"speed\":-1,\"duration_ms\":500}\n",
        "{\"type\":\"control\",\"cmd\":\"forward\",\"speed\":50}\n",
        "{\"type\":\"control\",\"cmd\":\"forward\",\"speed\":50,\"duration_ms\":0}\n",
        "{\"type\":\"control\",\"cmd\":\"forward\",\"speed\":50,\"duration_ms\":60001}\n",
        "{\"type\":\"control\",\"cmd\":\"stop\",\"cmd\":\"forward\"}\n",
        "{\"type\":\"position\",\"cmd\":\"stop\"}\n",
        "{\"type\":\"control\",\"cmd\":\"stop\"}garbage\n"
    };
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i) {
        feed(&link, bad[i]); assert(!esp_poll(&link, &command));
    }
    for (unsigned i = 0; i < ESP_BUFFER_SIZE + 100; ++i) esp_rx_byte(&link, 'x');
    assert(link.rx_overflow);
    assert(!esp_poll(&link, &command));
    feed(&link, "\n{\"type\":\"control\",\"cmd\":\"stop\"}\n");
    assert(esp_poll(&link, &command) && command.cmd == ESP_STOP);
    for (unsigned i = 0; i < 180; ++i) esp_rx_byte(&link, 'x');
    while (link.rx_tail != link.rx_head) assert(!esp_poll(&link, &command));
    feed(&link, "\n{\"type\":\"control\",\"cmd\":\"stop\"}\n");
    assert(esp_poll(&link, &command) && command.cmd == ESP_STOP);
    feed(&link, "{\"type\":\"control\",\"cmd\":\"stop\"}\n{\"type\":\"control\",\"cmd\":\"forward\",\"speed\":50,\"duration_ms\":500}\n");
    assert(esp_poll(&link, &command) && command.cmd == ESP_STOP);
    for (int i = 0; i < 3; ++i) {
        char text[160]; unsigned n = 0; uint8_t byte;
        assert(esp_position(&link, 31.2304f, 121.4737f, 90, .8f));
        while (esp_tx_byte(&link, &byte)) text[n++] = (char)byte;
        text[n] = 0;
        assert(strstr(text, "\"type\":\"position\"") && strstr(text, "\"speed\":0.800") && text[n-1] == '\n');
    }
    while (esp_position(&link, 0, 0, 0, 0)) {}
    assert(link.tx_overflow);                              /* 满队列仅丢帧，调用立即返回 */
    control_init(&car);
    assert(car.target == 20 && car.counts_per_meter == 3250);
    for (unsigned cmd = ESP_FORWARD; cmd <= ESP_BACKWARD; ++cmd) {
        command = (ESPCommand){(uint8_t)cmd, 50, 500};
        control_remote(&car, &command, UINT32_MAX - 100);
        control_step(&car, 0, 0, 0, 0, 0, 0, .01f, UINT32_MAX - 90, &clear, &lp, &rp);
        assert(car.running && car.phase == ROUTE_REMOTE);
        assert((lp > 0) == (cmd == ESP_FORWARD));
        assert((rp > 0) == (cmd == ESP_FORWARD));
        control_step(&car, 0, 0, 0, 0, 0, 0, .01f, 399, &clear, &lp, &rp);
        assert(!car.running && lp == 0 && rp == 0);         /* 毫秒计数折返后到期仍停车 */
    }
    for (unsigned cmd = ESP_LEFT; cmd <= ESP_RIGHT; ++cmd) {
        command = (ESPCommand){(uint8_t)cmd, 50, 500};
        control_remote(&car, &command, 0);
        control_step(&car, 0, 0, 0, 0, 0, 0, .01f, 10, &clear, &lp, &rp);
        assert(car.running && ((cmd == ESP_LEFT && lp < 0 && rp > 0) ||
                               (cmd == ESP_RIGHT && lp > 0 && rp < 0)));
        float target = cmd == ESP_LEFT ? 90.0f : -90.0f;
        control_step(&car, 0, 0, 0, 0, target, 0, .01f, 100, &clear, &lp, &rp);
        control_step(&car, 0, 0, 0, 0, target, 0, .01f, 200, &clear, &lp, &rp);
        control_step(&car, 0, 0, 0, 0, target, 0, .01f, 300, &clear, &lp, &rp);
        assert(!car.running && lp == 0 && rp == 0);
    }
    command = (ESPCommand){ESP_FORWARD, 50, 500};
    car.setting = SETTING_GPS; control_remote(&car, &command, 0); assert(!car.running);
    car.setting = SETTING_NONE; control_remote(&car, &command, 0);
    control_keys(&car, 0, 0, 1, 0, 10); assert(!car.running);
    control_remote(&car, &command, 20);
    command.cmd = ESP_STOP; control_remote(&car, &command, 30); assert(!car.running);
    puts("ESP32 protocol and remote control self-check: PASS");
    return 0;
}
