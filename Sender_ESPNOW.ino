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
#include "espnow_group.h"
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

group_config_t groupConfig = {0, 0, 0, DEFAULT_GROUP_RESPONSE_WINDOW_MS}; // Cấu hình nhóm mặc định xử lý ESPNOW
size_t configuredAssignmentCount = 0;                                     // Tổng số bản ghi gán nhóm do Hub lưu trữ sau khi đọc lệnh cấu hình
int configuredAssignmentMAClist[MAX_DEVICES][6] = {0};                       // Ds MAC đã được chỉ định nhóm theo cấu hình
uint8_t configuredAssignmentGroupId[MAX_DEVICES] = {0};                   // Nhóm tương ứng cho từng MAC trong mảng
static uint8_t currentRetryGroupId = 0;                                   // Tổng số bản ghi gán nhóm Hub lưu trữ sau khi đọc lệnh cấu hình
static unsigned long groupWindowStartMillis = 0;                          // thời điểm bắt đầu chờ phản hồi cho nhóm hiện tại, dùng để áp timeout theo group
uint32_t currentScanSessionId = 0;                                        // Bộ đếm phiên quét giúp xác định lần scan gần nhất của từng thiết bị

// Biến lưu cấu hình
int config_lid = 123;
int config_id = 2025;
bool config_received = false;
uint32_t nod = 0;      // số lượng thiết bị, cập nhật khi có node mới kết nối
uint32_t group_id = 0; // Nhóm hiện hành được gán cho thiết bị
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

// Khoảng thời gian chờ trước khi gửi lại yêu cầu trực tiếp tới từng node
static const unsigned long RESPONSE_RETRY_INTERVAL = 3000; // 3 giây
// Giới hạn số gói tin gửi lại trong mỗi vòng lặp để tránh nghẽn mạng
static const size_t MAX_DIRECT_RETRY_PER_LOOP = 5;
// Cờ đánh dấu đang chờ phản hồi sau khi đã broadcast
bool awaitingBroadcastResponses = false;

// Chuyển địa chỉ MAC sang dạng chuỗi "AA:BB:CC:DD:EE:FF" để tiện ghi log.
static String macToString(const uint8_t *mac)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

// Di chuyển sang nhóm kế tiếp khi nhóm hiện tại đã phản hồi xong.
static void advanceGroupWindowState(unsigned long nowMillis)
{
  if (currentRetryGroupId == 0)
  {
    return;
  }

  while (currentRetryGroupId <= groupConfig.groupCount)
  {
    if (hasPendingResponsesInGroup(currentRetryGroupId))
    {
      return;
    }
    uint8_t previousGroup = currentRetryGroupId;
    currentRetryGroupId++;
    groupWindowStartMillis = nowMillis;
    if (currentRetryGroupId <= groupConfig.groupCount)
    {
      Serial.printf("⏭️ Chuyển sang nhóm %u sau khi nhóm %u đã hoàn tất.\n",
                    currentRetryGroupId,
                    previousGroup);
    }
  }

  currentRetryGroupId = 0;
}

// Kiểm tra xem còn node nào chưa phản hồi hay không
static bool hasPendingResponses()
{
  for (int i = 0; i < Device.deviceCount; i++)
  {
    if (Device.pendingResponse[i])
    {
      return true;
    }
  }
  return false;
}

// Đánh dấu toàn bộ thiết bị đang chờ phản hồi sau khi đã broadcast yêu cầu.
// includeKnownDevices = false nghĩa là chỉ đánh dấu các thiết bị chưa từng phản hồi.
static bool markDevicesPendingForBroadcast(unsigned long nowMillis, bool includeKnownDevices)
{
  refreshGroupConfiguration();
  applyConfiguredGroupsToKnownDevices();

  unsigned int pendingCount = 0;
  unsigned int skippedCount = 0;

  for (int i = 0; i < Device.deviceCount; i++)
  {
    bool shouldAwait = includeKnownDevices;

    if (!includeKnownDevices)
    {
      shouldAwait = !Device.hasRespondedAtLeastOnce[i];
      if (!shouldAwait)
      {
        Device.pendingResponse[i] = false;
        Device.lastRequestMillis[i] = 0;
        Device.lastScanSessionId[i] = currentScanSessionId;
        skippedCount++;
        continue;
      }
    }

    Device.pendingResponse[i] = true;
    Device.lastRequestMillis[i] = nowMillis;
    Device.lastResponseMillis[i] = 0;
    ensureDeviceGroup(i);
    pendingCount++;
  }

  awaitingBroadcastResponses = pendingCount > 0;
  if (awaitingBroadcastResponses)
  {
    currentRetryGroupId = groupConfig.groupCount > 0 ? 1 : 0;
    groupWindowStartMillis = nowMillis;
    Serial.printf("⏳ Đang chờ phản hồi từ %u node đã biết...\n", pendingCount);
    Serial.printf("   • Tổng nhóm: %u, kích thước nhóm: %u, thời gian chờ mỗi nhóm: %lums\n",
                  groupConfig.groupCount,
                  groupConfig.groupSize,
                  static_cast<unsigned long>(groupConfig.responseWindowMs));
  }
  else
  {
    currentRetryGroupId = 0;
    groupWindowStartMillis = 0;
    if (!includeKnownDevices && skippedCount > 0)
    {
      Serial.printf("ℹ️ Bỏ qua yêu cầu phản hồi lại cho %u node đã được ghi nhận trước đó.\n", skippedCount);
    }
  }

  return awaitingBroadcastResponses;
}

// Bắt đầu một phiên quét mới. Có thể ép các thiết bị đã biết phản hồi lại khi includeKnownDevices = true.
static void startLicenseScan(bool includeKnownDevices)
{
  unsigned long nowMillis = millis();

  currentScanSessionId++;

  awaitingBroadcastResponses = false;
  currentRetryGroupId = 0;
  groupWindowStartMillis = 0;

  bool awaitingResponses = markDevicesPendingForBroadcast(nowMillis, includeKnownDevices);

  if (!awaitingResponses && includeKnownDevices && Device.deviceCount == 0)
  {
    Serial.println("ℹ️ Chưa có thiết bị nào được ghi nhận để thực hiện rescan.");
  }

  getlicense(Device_ID, WiFi.macAddress(), datalic.lid, nowMillis);
}

// Đảm bảo Hub đã đăng ký peer ESP-NOW trước khi gửi gói tin trực tiếp tới node.
static void ensurePeerRegistered(const uint8_t *mac_addr)
{
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac_addr, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST)
  {
    Serial.printf("❌ Không thể thêm peer %s (err=%d)\n",
                  macToString(mac_addr).c_str(),
                  result);
  }
}

// Gửi lại gói HUB_GET_LICENSE trực tiếp tới node chỉ định cùng thông tin nhóm.
static void getlicenseForMac(int id_des, const uint8_t *mac_des, int lid, unsigned long nowMillis)
{
  int opcode = LIC_GET_LICENSE;
  String macSrc = WiFi.macAddress();
  String macDesStr = macToString(mac_des);

  refreshGroupConfiguration();

  int index = findMacIndex(mac_des);
  uint8_t nodeGroupId = 0;
  if (index >= 0)
  {
    nodeGroupId = ensureDeviceGroup(index);
  }

  DynamicJsonDocument dataDoc(512);
  dataDoc["lid"] = lid;
  if (nodeGroupId != 0)
  {
    dataDoc["group_id"] = nodeGroupId;
  }

  appendGroupConfiguration(dataDoc, nodeGroupId, true); //Gửi kèm cấu hình để node cập nhật thứ tự phản hồi

  String output = createMessage(config_id, id_des, macSrc, macDesStr, opcode, dataDoc, nowMillis);

  if (output.length() > sizeof(message.payload))
  {
    Serial.printf("❌ Payload quá lớn khi gửi lại cho %s!\n", macDesStr.c_str());
    return;
  }

  memset(message.payload, 0, sizeof(message.payload));
  output.toCharArray(message.payload, sizeof(message.payload));

  ensurePeerRegistered(mac_des);
  esp_now_send(mac_des, (uint8_t *)&message, sizeof(message));

  Serial.printf("\n📤 Gửi HUB_GET_LICENSE trực tiếp tới %s (nhóm %u):\n",
                macDesStr.c_str(),
                nodeGroupId);
  Serial.println(output);
}

// Duyệt danh sách các node còn pending và gửi lại yêu cầu trực tiếp theo từng nhóm.
static void handlePendingResponses()
{
  if (!awaitingBroadcastResponses)
  {
    return;
  }

  unsigned long nowMillis = millis();
  refreshGroupConfiguration();

  bool hadAnyRequest = false;

  for (int i = 0; i < Device.deviceCount; i++)
  {
    if (Device.lastRequestMillis[i] != 0)
    {
      hadAnyRequest = true;
      break;
    }
  }

  advanceGroupWindowState(nowMillis);

  if (currentRetryGroupId != 0)
  {
    unsigned long elapsed = nowMillis - groupWindowStartMillis;
    if (elapsed < groupConfig.responseWindowMs)
    {
      return;
    }
  }

  size_t retrySent = 0;

  for (int i = 0; i < Device.deviceCount; i++)
  {
    if (!Device.pendingResponse[i])
    {
      continue;
    }

    uint8_t deviceGroup = ensureDeviceGroup(i);
    if (currentRetryGroupId != 0 && deviceGroup != currentRetryGroupId)
    {
      continue;
    }

    if (nowMillis - Device.lastRequestMillis[i] < RESPONSE_RETRY_INTERVAL)
    {
      continue;
    }

    getlicenseForMac(Device.DeviceID[i], Device.MACList[i], Device.LocalID[i], nowMillis);
    Device.lastRequestMillis[i] = nowMillis;
    retrySent++;
    hadAnyRequest = true;

    if (retrySent >= MAX_DIRECT_RETRY_PER_LOOP)
    {
      break;
    }
  }

  if (retrySent > 0 && currentRetryGroupId != 0)
  {
    groupWindowStartMillis = nowMillis;
  }

  if (!hasPendingResponses())
  {
    awaitingBroadcastResponses = false;
    currentRetryGroupId = 0;
    if (!hadAnyRequest)
    {
      return;
    }

    if (retrySent > 0)
    {
      Serial.println("✅ Tất cả node đã phản hồi sau lần gửi lại.");
    }
    else
    {
      Serial.println("✅ Tất cả node đã phản hồi.");
    }
  }
  else
  {
    advanceGroupWindowState(nowMillis);
  }
}

void sendGetLicenseBroadcast(int id_des, const String &mac_src, int lid, unsigned long nowMillis)
{
  int opcode = LIC_GET_LICENSE;
  String mac_des = WiFi.macAddress();
  int id_src = config_id;
  refreshGroupConfiguration();
  for (int i = 0; i < Device.deviceCount; i++)
  {
    ensureDeviceGroup(i);
  }

  bool includeAssignments = true;
  String output;

  DynamicJsonDocument dataDoc(128);
  dataDoc["lid"] = lid;

  if (output.length() > sizeof(message.payload))
  {
    Serial.println("❌ Payload quá lớn!");
    return;
  }
  memset(message.payload, 0, sizeof(message.payload));
  output.toCharArray(message.payload, sizeof(message.payload));
  esp_now_send(receiverMac, (uint8_t *)&message, sizeof(message));

  Serial.println("\n📤 Gửi HUB_GET_LICENSE broadcast:");
  Serial.println(output);

  // Sau khi broadcast, đánh dấu toàn bộ node là đang chờ phản hồi
  markDevicesPendingForBroadcast(nowMillis, true);
}

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
  serial_pc();
  led.update(); // Gọi liên tục trong loop()

  // Kiểm tra và gửi lại yêu cầu cho các node chưa phản hồi
  handlePendingResponses();

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
    case 2:
      Serial.println("Gửi lệnh LIC_CONFIG_DEVICE");
      config_device(Device_ID, datalic.lid, WiFi.macAddress(), nod, millis());
      break;
    case 4:
      Serial.println("Gửi lệnh LIC_GET_LICENSE_RECAN");
      startLicenseScan(true);
      break;
    case 5:
      Serial.println("Gửi lệnh LIC_GET_LICENSE_SCAN");
      startLicenseScan(false);
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
        if (Device.DeviceID[j] == Device.DeviceID[i])
        {
          count++;
        }
      }
      snprintf(nodStr, sizeof(nodStr), "%d", count);

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
