#include <Arduino.h>
#include "stdio.h"
#include <esp_display_panel.hpp>
#include "WiFi.h"
#include "ESP32_NOW.h"
#include <ArduinoJson.h>
#include <MD5Builder.h>
#include <time.h>
#include "led_status.h"

// Thư viện LVGL
#include <lvgl.h>
#include "lvgl_v8_port.h"
#include "ui.h"
#include "function.h"

#include "config.h"
#include "espnow_handler.h"
#include "protocol_handler.h"
#include "serial.h"

// Biến toàn cục
LedStatus led(LED_PIN); // LED nối chân 2
LicenseInfo globalLicense;
PayloadStruct message;
LicenseInfo licenseInfo; // Biến lưu thông tin license
device_info Device;
char jsonBuffer[BUFFER_SIZE];
int bufferIndex;
char messger[128];
uint8_t button = 0;

// Biến lưu cấu hình
int config_lid = 123;
int config_id = 2025;
bool config_received = false;
uint32_t nod = 0; //số lượng thiết bị, cập nhật khi có node mới kết nối
int next_page = 0;
int old_page = 0;

// Biến LED
bool errorState = false;
unsigned long previousMillis = 0;
const long blinkInterval = 500;
bool ledState = LOW;

// Biến thời gian license
time_t start_time = 0;
const uint32_t duration = 60; // 60 phút
uint32_t now = 0;

// Biến expired
bool expired_flag = false; // Biến logic kiểm soát trạng thái
// int expired = 1;           // 0 = chưa hết hạn, 1 = hết hạn
uint8_t expired = expired_flag ? 1 : 0; // 1 là hết hạn, 0 là còn hạn

// bool reported_before = false; // Đánh dấu đã từng phản hồi

// Nhận gói Scan
//  void onScanRequest()
//  {
//      if (!reported_before)
//      {
//          sendScanResponse(); // Gửi phản hồi về Master
//          reported_before = true;
//          Serial.println("Đã phản hồi SCAN");
//      }
//      else
//      {
//          Serial.println("Đã từng phản hồi trước đó, bỏ qua");
//      }
//  }

// Rescan
//  void onResetScanState()
//  {
//      reported_before = false; // Reset trạng thái để phản hồi lại lần scan tiếp theo
//      Serial.println("Reset trạng thái: sẽ phản hồi SCAN tiếp theo");
//  }

void setup()
{
  String title = "LVGL porting example";
  Serial.begin(115200);
  Serial.println("Initializing board");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("❌ ESP-NOW init failed!");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6); // FF:FF:FF:FF:FF:FF
  peerInfo.channel = 1;                       // Kênh cố định để đồng bộ với sender
  peerInfo.encrypt = false;                   // tạm thời tắt mã hóa
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("❌ Failed to add peer!");
  }
  else
    Serial.println("add peer ok");

  configTime(0, 0, "pool.ntp.org");

  led.setState(CONNECTION_ERROR);

  Board *board = new Board();
  board->init();

#if LVGL_PORT_AVOID_TEARING_MODE
  auto lcd = board->getLCD();
  // When avoid tearing function is enabled, the frame buffer number should be set in the board driver
  lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
  auto lcd_bus = lcd->getBus();
  /**
   * As the anti-tearing feature typically consumes more PSRAM bandwidth, for the ESP32-S3, we need to utilize the
   * "bounce buffer" functionality to enhance the RGB data bandwidth.
   * This feature will consume `bounce_buffer_size * bytes_per_pixel * 2` of SRAM memory.
   */
  if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB)
  {
    static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
  }
#endif
#endif
  assert(board->begin());

  Serial.println("Initializing LVGL");
  lvgl_port_init(board->getLCD(), board->getTouch());

  Serial.println("Creating UI");
  /* Lock the mutex due to the LVGL APIs are not thread-safe */
  lvgl_port_lock(-1);
  // Khởi tạo ui.
  ui_init();
  /* Release the mutex */
  lvgl_port_unlock();
}
bool ledstt = 1;
// time_t now;
unsigned long nowMillis = millis();
// static unsigned long lastSendTime = 0;
void loop()
{

  // lv_timer_handler();
  // Serial.println("IDLE loop");
  // delay(1000);
  // if (datalic.expired)
  // {
  //     Serial.printf("Local ID: %d\n\r",datalic.lid);
  //     Serial.printf("Device ID: %d\n\r",Device_ID);
  //     Serial.printf("Duration: %d\n\r",datalic.duration);
  //     datalic.expired=false;
  // }
  // }
  serial_pc();
  led.update(); // Gọi liên tục trong loop()

  if (button != 0)
  {
    Serial.println("button pressed: ");
    Serial.println(button);
    switch (button)
    {
    case 1:
      Serial.println("Gửi lệnh LIC_SET_LICENSE");
      set_license(Device_ID, datalic.lid, WiFi.macAddress(), millis(), datalic.duration, 1, millis());
      break;
    case 4:
      Serial.println("Gửi lệnh LIC_GET_LICENSE_SCAN");
      getlicense(Device_ID, WiFi.macAddress(), datalic.lid, millis());
      break;
    case 5:
      Serial.println("Gửi lệnh LIC_GET_LICENSE_RESCAN");
      memset(&Device, 0, sizeof(Device));
      getlicense(Device_ID, WiFi.macAddress(), datalic.lid, millis());
      break;
    default:
      break;
    }
    button = 0;
  }
  // delay(10);

  if (enable_print_ui_set)
  {
    lvgl_port_lock(-1);
    lv_obj_t *OBJ_Notification = add_Notification(ui_SCRSetLIC, messger);
    lvgl_port_unlock();
    enable_print_ui_set = false;
  }

  if (enable_print_ui || (old_page != next_page))
  {

    lvgl_port_lock(-1);
    if (timer != NULL)
    {
      lv_timer_del(timer);
      timer = NULL;
    }

    if (ui_spinner1 != NULL)
    {
      lv_obj_del(ui_spinner1);
      ui_spinner1 = NULL;
    }

    lv_obj_clean(ui_Groupdevice);
    lv_obj_invalidate(ui_Groupdevice);

    if ((next_page * maxLinesPerPage) >= Device.deviceCount)
    {
      next_page = 0; // Quay về trang đầu
    }

    old_page = next_page;
    int startIdx = next_page * maxLinesPerPage;
    int endIdx = startIdx + maxLinesPerPage;
    if (endIdx > Device.deviceCount)
      endIdx = Device.deviceCount;
    if (startIdx >= endIdx || startIdx < 0 || endIdx > MAX_DEVICES)
    {
      enable_print_ui = false;
      lvgl_port_unlock();
      return;
    }
    Serial.printf("%d %d %d \n ", startIdx, endIdx, next_page);

    if (ui_Groupdevice)
    {
      /* Hide container while rebuilding to prevent visible tearing */
      lv_obj_add_flag(ui_Groupdevice, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clean(ui_Groupdevice);
      // lv_obj_invalidate(ui_Groupdevice);
    }

    if (ui_Label7)
    {
      char buf[64];
      snprintf(buf, sizeof(buf), "LIST DEVICE: %2d - Page %2d", Device.deviceCount, next_page + 1);
      lv_label_set_text(ui_Label7, buf);
      Serial.printf("BUF= %s\n", buf);
    }

    for (int i = startIdx; i < endIdx; i++)
    {
      if (i < 0 || i >= MAX_DEVICES)
        continue;
      if (Device_ID != 0 && Device.DeviceID[i] != Device_ID)
        continue;
      char macStr[18], idStr[18], lidStr[18], timeStr[18], nodStr[8];

      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
               Device.MACList[i][0], Device.MACList[i][1], Device.MACList[i][2],
               Device.MACList[i][3], Device.MACList[i][4], Device.MACList[i][5]);

      snprintf(idStr, sizeof(idStr), "%d", Device.DeviceID[i]);
      snprintf(lidStr, sizeof(lidStr), "%d", Device.LocalID[i]);
      snprintf(timeStr, sizeof(timeStr), "%lu", Device.timeLIC[i]); // chuyển %d = %lu

      int count = 0;
      for (int j = 0; j < Device.deviceCount; j++)
      {
        if (Device.LocalID[j] == Device.LocalID[i])
        {
          count++;
        }
      }

      lv_obj_t *ui_DeviceINFO = ui_DeviceINFO1_create(ui_Groupdevice, idStr, lidStr, nodStr, macStr, timeStr);
    }
    if (ui_Groupdevice)
    {
      /* Reveal the list once all items are created and update layout */
      lv_obj_clear_flag(ui_Groupdevice, LV_OBJ_FLAG_HIDDEN);
      lv_obj_invalidate(ui_Groupdevice);
    }
    enable_print_ui = false;
    lvgl_port_unlock();
  }
}
