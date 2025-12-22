#ifndef PROTOCOL_HANDLER_H
#define PROTOCOL_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESP32_NOW.h>
#include <ArduinoJson.h>
#include <stdio.h>
// #include <deque>

#include "config.h"
#include "led_status.h"
#include "espnow_handler.h"
#include "serial.h"
#include "function.h"
#include "espnow_group.h"

void addMacToList(int id, int lid, const uint8_t *mac_addr, unsigned long time_, uint8_t groupId);

static String formatMacAddress(const uint8_t mac[6]);

// Tạo tin nhắn phản hồi
String createMessage(int id_src, int id_des, String mac_src, String mac_des, uint8_t opcode, const DynamicJsonDocument &data, unsigned long timestamp = 0)
{
  if (timestamp == 0)
  {
    timestamp = millis() / 1000; // Giả lập Unix time (cần đồng bộ thực tế)
  }

  String dataStr;
  serializeJson(data, dataStr);

  String auth = md5Hash(id_src, id_des, mac_src, mac_des, opcode, dataStr, timestamp);
  DynamicJsonDocument message(512);
  message["id_src"] = id_src;
  message["id_des"] = id_des;
  message["mac_src"] = mac_src;
  message["mac_des"] = mac_des;
  message["opcode"] = opcode;
  message["data"] = data;
  message["time"] = timestamp;
  message["auth"] = auth;

  String messageStr;
  serializeJson(message, messageStr); // Chuyển thành chuỗi JSON
  return messageStr;
}

// Xử lý phản hồi từ ESP NOW
void processReceivedData(StaticJsonDocument<512> message, const uint8_t *mac_addr)
{
  int id_src = message["id_src"];
  int id_des = message["id_des"];
  // String mac_src = message["mac_src"];
  // String mac_des = message["mac_des"];

  String mac_src;
  if (message.containsKey("mac_src") && !message["mac_src"].isNull())
  {
    mac_src = message["mac_src"].as<String>();
  }
  else
  {
    mac_src = formatMacAddress(mac_addr);
  }

  String mac_des;
  if (message.containsKey("mac_des") && !message["mac_des"].isNull())
  {
    mac_des = message["mac_des"].as<String>();
  }
  else
  {
    mac_des = WiFi.macAddress();
  }

  uint8_t opcode = message["opcode"];
  String dataStr;
  serializeJson(message["data"], dataStr);
  unsigned long timestamp = message["time"];
  String receivedAuth = message["auth"];

  String calculatedAuth = md5Hash(id_src, id_des, mac_src, mac_des, opcode, dataStr, timestamp);

  if (!receivedAuth.equalsIgnoreCase(calculatedAuth))
  {
    Serial.println("❌ Lỗi xác thực: Mã MD5 không khớp!");
    return;
  }
  else
  {
    Serial.println("MD5 OK!");
  }
  serializeJson(message, Serial);

  // Chỉ xử lý gói tin gửi tới đúng Hub hiện tại để tránh trộn dữ liệu truy vấn
  if (id_des != config_id)
  {
    Serial.printf("⚠️ Bỏ qua gói tin dành cho id_des=%d (Hub hiện tại=%d)\n", id_des, config_id);
    return;
  }

  switch (opcode)
  {
  case LIC_SET_LICENSE | 0x80:
  {
    JsonObject data = message["data"];
    int lid = data["lid"];
    int Status = data["status"];
    const char *error_msg = data["error_msg"].as<const char *>();

    sprintf(messger, "Status: %d \nLocal ID: %d\n", Status, lid); // Đổi %s sang %d
    if (error_msg != NULL)
    {
      // strcat(messger, "Lỗi: ");
      // strcat(messger, error_msg);
      strncat(messger, "Lỗi: ", sizeof(messger) - strlen(messger) - 1);
      strncat(messger, error_msg, sizeof(messger) - strlen(messger) - 1);
    }

    // enable_print_ui_set=true;
    // timer_out=millis();
    Serial.println("== Đã nhận phản hồi Data Object ==");
    Serial.print("LID: ");
    Serial.println(lid);
    Serial.print("Status: ");
    Serial.println(Status);
    break;
  }

  case LIC_GET_LICENSE | 0x80:
  {
    // Serial.println("Đã nhận phản hồi HUB_GET_LICENSE:");
    JsonObject data = message["data"];
    int lid = data["lid"];
    unsigned long time_temp = data["remain"];

    uint8_t groupId = 0;
    if (data.containsKey("group_id"))
    {
      groupId = data["group_id"].as<uint8_t>();
    }

    addMacToList(id_src, lid, mac_addr, time_temp, groupId);
    break;
  }

  case LIC_CONFIG_DEVICE | 0x80:
  {
    // Serial.println("Đã nhận phản hồi LIC_CONFIG_DEVICE:");
    JsonObject data = message["data"];
    int new_id = data["id"];
    int new_lid = data["lid"];
    int nod = data["nod"];
    int status = data["status"];
    const char *error_msg = data["error_msg"].as<const char *>();

    sprintf(messger, "Status: %d \nDevice ID: %d\nLocal ID: %d\n", status, new_id, new_lid);
    if (error_msg != NULL)
    {
      strncat(messger, "Lỗi: ", sizeof(messger) - strlen(messger) - 1);
      strncat(messger, error_msg, sizeof(messger) - strlen(messger) - 1);
    }

    Serial.print("Device ID: ");
    Serial.println(new_id);
    Serial.print("LID: ");
    Serial.println(new_lid);
    Serial.print("NOD: ");
    Serial.println(nod);
    Serial.print("Status: ");
    Serial.println(status);
    if (error_msg != NULL)
    {
      Serial.print("Lỗi: ");
      Serial.println(error_msg);
    }
    break;
  }

  default:
    // if (opcode != 0x83) {  // Bỏ qua opcode 0x83
    Serial.printf("Unknown opcode: 0x%02X\n", opcode);
    // }
    break;
  }
}

//======================================

// Chuyển địa chỉ MAC sang dạng chuỗi "AA:BB:CC:DD:EE:FF" để tiện ghi log.
static String macToString(const uint8_t *mac)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

// Đảm bảo Hub đã đăng ký peer ESP-NOW trước khi gửi gói tin tt tới node.
static void ensurePeerRegistered(const uint8_t *mac_addr)
{
  if (esp_now_is_peer_exist(mac_addr)) return;

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


static String formatMacAddress(const uint8_t mac[6])
{
  char macBuffer[18];
  snprintf(macBuffer, sizeof(macBuffer), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macBuffer);
}

void set_license(int id_des, int lid, String mac_des, time_t created, uint32_t duration, uint8_t expired, uint32_t now)
{
  int opcode = LIC_SET_LICENSE;
  String mac = WiFi.macAddress();
  int id_src = config_id;
  DynamicJsonDocument dataDoc(256);

  dataDoc["lid"] = lid;
  dataDoc["created"] = created;
  dataDoc["duration"] = duration;
  dataDoc["expired"] = expired;

  String output = createMessage(id_src, id_des, mac, mac_des, opcode, dataDoc, now);

  if (output.length() > sizeof(message.payload))
  {
    Serial.println("❌ Payload quá lớn! 1");
    return;
  }

  output.toCharArray(message.payload, sizeof(message.payload));
  esp_now_send(receiverMac, (uint8_t *)&message, sizeof(message));

  Serial.println("\n📤 Gửi HUB_SET_LICENSE:");
  Serial.println(output);
}

// Gửi HUB_GET_LICENSE
void getlicense(int id_des, String mac_des, int lid, unsigned long now)
{
  int opcode = LIC_GET_LICENSE;
  String mac = WiFi.macAddress();
  int id_src = config_id;
  DynamicJsonDocument dataDoc(512);
  dataDoc["lid"] = lid;

  refreshGroupConfiguration();
  uint8_t targetGroupId = findLowestPendingGroupId();
  if (targetGroupId != 0)
  {
    dataDoc["group_id"] = targetGroupId;
  }
  // appendGroupConfiguration(dataDoc, targetGroupId, true);
  String output = createMessage(id_src, id_des, mac, mac_des, opcode, dataDoc, now);

  if (output.length() > sizeof(message.payload))
  {
    Serial.println("❌ Payload quá lớn! 2");
    return;
  }

  output.toCharArray(message.payload, sizeof(message.payload));
  esp_now_send(receiverMac, (uint8_t *)&message, sizeof(message));

  Serial.println("📤 Gửi HUB_GET_LICENSE:");
  Serial.println(output);
}

// Gửi lại gói HUB_GET_LICENSE trực tiếp tới node chỉ định cùng thông tin nhóm.
static void getlicenseForMac(int id_des, const uint8_t *mac_des, int lid, unsigned long nowMillis)
{
  int opcode = LIC_GET_LICENSE;
  String macSrc = WiFi.macAddress();
  String macDesStr = macToString(mac_des);

  // refreshGroupConfiguration();
  recalcAndApplyGroupConfiguration();

  // int index = findMacIndex(mac_des);
  // uint8_t nodeGroupId = 0;
  // if (index >= 0)
  // {
  //   nodeGroupId = ensureDeviceGroup(index);
  // }
  uint8_t nodeGroupId = ensureDeviceGroupForMac(mac_des);
  DynamicJsonDocument dataDoc(512);
  dataDoc["lid"] = lid;
  if (nodeGroupId != 0)
  {
    dataDoc["group_id"] = nodeGroupId;
  }

  // appendGroupConfiguration(dataDoc, nodeGroupId, false); // Gửi kèm cấu hình để node cập nhật thứ tự phản hồi

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


/////////////////////////////////////

bool parseMac(const String &macStr, uint8_t mac[6])
{
  int values[6];
  if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x",
             &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5]) != 6)
  {
    return false;
  }
  for (int i = 0; i < 6; i++)
    mac[i] = (uint8_t)values[i];
  return true;
}

void config_device(int id_des, int lid, String mac_des, uint32_t nod, unsigned long now)
{
  int opcode = LIC_CONFIG_DEVICE;
  String mac = WiFi.macAddress();
  int id_src = config_id;
  DynamicJsonDocument dataDoc(128);
  dataDoc["id"] = id_des;
  dataDoc["lid"] = lid;
  dataDoc["nod"] = nod;
  // appendGroupConfiguration(dataDoc, 0, false);

  String output = createMessage(id_src, id_des, mac, mac_des, opcode, dataDoc, now);

  if (output.length() > sizeof(message.payload))
  {
    Serial.println("❌ Payload quá lớn! 3");
    return;
  }

  uint8_t destMac[6];
  if (!parseMac(mac_des, destMac))
  {
    Serial.println("❌ MAC đích sai format");
    return;
  }

  ensurePeerRegistered(destMac);

  output.toCharArray(message.payload, sizeof(message.payload));

  esp_err_t r = esp_now_send(destMac, (uint8_t *)&message, sizeof(message));
  if (r != ESP_OK)
  {
    Serial.printf("❌ esp_now_send failed: %d\n", r);
    return;
  }

  // output.toCharArray(message.payload, sizeof(message.payload));
  // esp_now_send(receiverMac, (uint8_t *)&message, sizeof(message));

  Serial.println("📤 Gửi LIC_CONFIG_DEVICE:");
  Serial.println(output);
}

//---------------------Đưa vào hàm gọi group
group_config_t groupConfig = {0, 0, 0, DEFAULT_GROUP_RESPONSE_WINDOW_MS}; // Cấu hình nhóm mặc định xử lý ESPNOW
static uint8_t currentRetryGroupId = 0;                                   // Tổng số bản ghi gán nhóm Hub lưu trữ sau khi đọc lệnh cấu hình
static unsigned long groupWindowStartMillis = 0;                          // thời điểm bắt đầu chờ phản hồi cho nhóm hiện tại, dùng để áp timeout theo group
uint32_t currentScanSessionId = 0;             

// Khoảng thời gian chờ trước khi gửi lại yêu cầu trực tiếp tới từng node
static const unsigned long RESPONSE_RETRY_INTERVAL = 3000; // 3 giây
// Giới hạn số gói tin gửi lại trong mỗi vòng lặp để tránh nghẽn mạng
static const size_t MAX_DIRECT_RETRY_PER_LOOP = 3;
// Cờ đánh dấu đang chờ phản hồi sau khi đã broadcast
bool awaitingBroadcastResponses = false;

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
static bool markDevicesPendingForBroadcast(unsigned long nowMillis, bool includeKnownDevices, unsigned int *outSkippedKnownDevices = nullptr)
{
  // refreshGroupConfiguration();
  // applyConfiguredGroupsToKnownDevices();
  recalcAndApplyGroupConfiguration();

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
        // Device.lastScanSessionId[i] = currentScanSessionId;
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
  if (outSkippedKnownDevices)
  {
    *outSkippedKnownDevices = skippedCount;
  }
  if (awaitingBroadcastResponses)
  {
    currentRetryGroupId = groupConfig.groupCount > 0 ? 1 : 0;
    groupWindowStartMillis = nowMillis;
    Serial.printf("⏳ Đang chờ phản hồi từ %u node đã biết...\n", pendingCount);
    Serial.printf("   • Tổng nhóm: %u, số node trong nhóm: %u, thời gian chờ mỗi nhóm: %lums\n",
                  groupConfig.groupCount,
                  groupConfig.groupSize,
                  static_cast<unsigned long>(groupConfig.responseWindowMs));
  }
  else
  {
    currentRetryGroupId = 0;
    groupWindowStartMillis = 0;
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

  // bool awaitingResponses = markDevicesPendingForBroadcast(nowMillis, includeKnownDevices);

  // if (!awaitingResponses && includeKnownDevices && Device.deviceCount == 0)
  // {
  //   Serial.println("ℹ️ Chưa có thiết bị nào được ghi nhận để thực hiện rescan.");
  // }

  unsigned int skippedKnownDevices = 0;
  bool awaitingResponses = markDevicesPendingForBroadcast(nowMillis,
                                                          includeKnownDevices,
                                                          &skippedKnownDevices);

  // if (!awaitingResponses && !includeKnownDevices && skippedKnownDevices > 0)
  // {
  //   Serial.printf("ℹ️ Đã ghi nhận %u node từ các lần quét trước, không có node mới trong đợt này.\n",
  //                 skippedKnownDevices);
  //   Serial.println("🔁 Tự động yêu cầu các node đã biết phản hồi lại để cập nhật thay đổi.");
  //   includeKnownDevices = true;
  //   awaitingResponses = markDevicesPendingForBroadcast(nowMillis, true);
  // }

  if (!awaitingResponses && includeKnownDevices && Device.deviceCount == 0)
  {
    Serial.println("ℹ️ Chưa có thiết bị nào được ghi nhận để thực hiện rescan.");
  }

  bool includeAssignments = awaitingResponses;
  getlicense(Device_ID, WiFi.macAddress(), datalic.lid, nowMillis);
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

// Duyệt danh sách các node còn pending và gửi lại yêu cầu trực tiếp theo từng nhóm.
static void handlePendingResponses()
{
  if (!awaitingBroadcastResponses)
  {
    return;
  }

  unsigned long nowMillis = millis();
  // refreshGroupConfiguration();
  recalcAndApplyGroupConfiguration();

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

#endif
