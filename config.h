#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <time.h>
#include <MD5Builder.h>
#include <WiFi.h>
// #include <Preferences.h> // Thư viện để lưu trữ dữ liệu vào bộ nhớ flash
#include <esp_display_panel.hpp>
#include "ESP32_NOW.h"
#include "led_status.h"
#include <lvgl.h>
#include "lvgl_v8_port.h"
#include "ui.h"
#include "function.h"


using namespace esp_panel::drivers;
using namespace esp_panel::board;
Licence datalic;
int Device_ID;

uint32_t timer_out = 0;
bool enable_print_ui = false;
bool enable_print_ui_set = false;
lv_timer_t *timer = NULL;

#define LED_PIN 2       // Chân LED báo trạng thái

#define MASTER__ID 1001 // ID của thiết bị chính (Hub66s)
#define maxLinesPerPage 20

// Các opcode cho các lệnh
#define LIC_TIME_GET 0x01
#define LIC_SET_LICENSE 0x02        // tạo bản tin license
#define LIC_GET_LICENSE 0x03        // đọc trạng thái license của Hub66s
#define LIC_LICENSE_DELETE 0x04     // xóa 1 license
#define LIC_LICENSE_DELETE_ALL 0x05 // xóa tất cả license
#define LIC_INFO 0x06               // cập nhật thông tin Lic66s
#define LIC_CONFIG_DEVICE 0x07      // cấu hình thông tin thiết bị
#define LIC_INFO_RESPONSE 0x80      // 0x06 | 0x80

// Biến dùng trong hàm nhận dữ liệu json từ PC
const int BUFFER_SIZE = 512; // Tăng kích thước buffer để chứa JSON lớn
//-----------------------------------------------
uint8_t receiverMac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // Broadcast
const char *private_key = "khoabi_mat_123";

String getDeviceMacAddress()
{
    // Lấy địa chỉ Mac của Wifi station (client)
    return WiFi.macAddress();
}

typedef struct
{
    int lid;        // License ID
    int id;         // ID thiết bị
    String license; // Nội dung license
    time_t created;
    time_t expired;
    uint32_t duration;
    uint32_t remain;
    bool expired_flag; // Đã hết hạn chưa
    String deviceName; // Tên thiết bị
    String version;    // Phiên bản firmware
} LicenseInfo;

typedef struct
{
    char payload[250];
} PayloadStruct;

// Ham mã hóa Auth MD5
String md5Hash(int id_src, int id_des, const String &mac_src, const String &mac_des, uint8_t opcode, 
    const String &data, unsigned long timestamp)
{
    MD5Builder md5;
    md5.begin();
    md5.add(String(id_src));
    md5.add(String(id_des));
    md5.add(mac_src);
    md5.add(mac_des);
    md5.add(String(opcode));
    md5.add(data);
    md5.add(String(timestamp));
    md5.add(private_key);
    md5.calculate();
    return md5.toString(); // Trả về chuỗi MD5 hex
}

//Json playload
extern bool config_received;

//Biến toàn cục 
extern LedStatus led;
extern LicenseInfo globalLicense;
extern PayloadStruct message;
extern int config_id;
extern int config_lid;
extern int id_des;
extern bool config_processed;
extern char jsonBuffer[BUFFER_SIZE];
extern int bufferIndex;
extern time_t start_time;
extern const uint32_t duration;
extern bool expired_flag;
extern uint8_t expired;
extern uint32_t now;
extern uint32_t lastSendTime;
extern String device_id;
extern uint32_t nod; // number of device

#endif // CONFIG_H