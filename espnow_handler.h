#ifndef ESPNOW_HANDLER_H
#define ESPNOW_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include "ESP32_NOW.h"
#include <ArduinoJson.h>
#include <cstring>

#include "config.h"
#include "espnow_group.h"
#include "protocol_handler.h"

/**
 * @brief Áp dụng group id node báo về, hoặc sử dụng cấu hình Hub nếu thiếu.
 *
 * @param index            Vị trí thiết bị trong danh sách Device.
 * @param expectedGroupId  Nhóm Hub đã cấu hình cho thiết bị.
 * @param reportedGroupId  Nhóm mà node phản hồi về (0 nếu không gửi).
 * @return true            Nếu Hub phải sử dụng lại cấu hình cục bộ thay vì giá trị node cung cấp.
 */
inline bool applyReportedGroupToDevice(int index, uint8_t expectedGroupId, uint8_t reportedGroupId)
{
  if (reportedGroupId == 0)
  {
    Device.groupId[index] = expectedGroupId;
    return true;
  }

  if (reportedGroupId != expectedGroupId)
  {
    Device.groupId[index] = expectedGroupId;
    return true;
  }

  Device.groupId[index] = reportedGroupId;
  return false;
}

void logDeviceStatus(const char *prefix, const uint8_t *mac_addr, int index, bool usedHubGroupFallback, uint8_t reportedGroupId)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  Serial.print(prefix);
  Serial.print(" | MAC: ");
  Serial.print(macStr);
  Serial.print(" | Group: ");
  Serial.print(Device.groupId[index]);
  Serial.print(" | Responses: ");
  Serial.println(Device.responseCount[index]);

  if (usedHubGroupFallback)
  {
    if (reportedGroupId == 0)
    {
      Serial.println("⚠️ Node không gửi group_id, áp dụng nhóm theo cấu hình hiện tại.");
    }
    else
    {
      Serial.printf("⚠️ Node báo group_id=%u, áp dụng nhóm %u theo cấu hình hiện tại.\n",
                    reportedGroupId,
                    Device.groupId[index]);
    }
  }
}

// Tìm vị trí của MAC trong danh sách hiện có, trả về -1 nếu không tìm thấy
int findMacIndex(const uint8_t *mac_addr)
{
  for (int i = 0; i < Device.deviceCount; i++)
  {
    if (memcmp(Device.MACList[i], mac_addr, 6) == 0)
    {
      return i;
    }
  }
  return -1;
}

// Thêm mới hoặc cập nhật node dựa trên phản hồi nhận được từ
void addMacToList(int id, int lid, const uint8_t *mac_addr, unsigned long time_, uint8_t groupId)
{
  // Kiểm tra xem node đã có trong danh sách hay chưa
  int existingIndex = findDeviceIndexByMac(mac_addr);

  refreshGroupConfiguration(); // Sau khi đọc cấu hình mới nhất, mọi phép gán nhóm sẽ được cập nhật trước khi xử lý phản hồi.

  if (existingIndex != -1)
  {
    uint8_t expectedGroupId = resolveGroupIdForIndex(existingIndex);
    bool fallbackToConfiguredGroup = applyReportedGroupToDevice(existingIndex, expectedGroupId, groupId);

    Device.DeviceID[existingIndex] = id;
    Device.LocalID[existingIndex] = lid;
    Device.timeLIC[existingIndex] = time_;
    // Cập nhật thống kê phản hồi khi thiết bị trả lời
    Device.responseCount[existingIndex]++;
    Device.lastResponseMillis[existingIndex] = millis();
    Device.pendingResponse[existingIndex] = false;
    Device.hasRespondedAtLeastOnce[existingIndex] = true;
    Device.lastScanSessionId[existingIndex] = currentScanSessionId;

    logDeviceStatus("🔄 Cập nhật thiết bị", mac_addr, existingIndex, fallbackToConfiguredGroup, groupId);
    return;
  }
  uint32_t configuredGroupLimit = static_cast<uint32_t>(resolveRequestedGroupCount()) * resolveRequestedGroupSize();
  uint32_t computedGroupLimit = static_cast<uint32_t>(groupConfig.groupCount) * groupConfig.groupSize;

  uint32_t maxAllowedDevices = configuredGroupLimit != 0 ? configuredGroupLimit : computedGroupLimit;

  if (maxAllowedDevices != 0 && Device.deviceCount >= maxAllowedDevices)
  {
    Serial.printf("Không thể thêm thiết bị mới: đã đạt giới hạn %lu thiết bị theo cấu hình nhóm.\n",
                  static_cast<unsigned long>(maxAllowedDevices));
    return;
  }

  if (Device.deviceCount >= MAX_DEVICES)
  {
    Serial.println("Không thể thêm thiết bị mới: đã đạt giới hạn.");
    return;
  }

  // Thêm node mới vào cuối danh sách
  int index = Device.deviceCount;
  memcpy(Device.MACList[index], mac_addr, 6);

  Device.DeviceID[index] = id;
  Device.LocalID[index] = lid;
  Device.timeLIC[index] = time_;
  Device.responseCount[index] = 1;             // Lần phản hồi đầu tiên
  Device.lastResponseMillis[index] = millis(); // Ghi nhận thời điểm phản hồi
  Device.lastRequestMillis[index] = 0;         // Chưa có lần yêu cầu đơn lẻ nào
  Device.pendingResponse[index] = false;       // Đã phản hồi cho lượt broadcast hiện tại
  uint8_t expectedGroupId = resolveGroupIdForIndex(index);
  bool fallbackToConfiguredGroup = applyReportedGroupToDevice(index, expectedGroupId, groupId);
  Device.hasRespondedAtLeastOnce[index] = true;
  Device.lastScanSessionId[index] = currentScanSessionId;
  Device.deviceCount++;

  logDeviceStatus("Thêm thiết bị", mac_addr, index, fallbackToConfiguredGroup, groupId);
}

void printDeviceList()
{
  Serial.println("Danh sách thiết bị đã tìm thấy:");
  for (int i = 0; i < Device.deviceCount; i++)
  {
    char prefix[24];
    snprintf(prefix, sizeof(prefix), "📋 Thiết bị %d", i + 1);
    logDeviceStatus(prefix, Device.MACList[i], i, false, 0);
  }
  Serial.println("------------------");
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Send Status: ");

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}
// Bộ xử lý chung cho toàn bộ gói tin ESP-NOW trả về từ node
void onReceive(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len)
{
  const uint8_t *mac_addr = recv_info->src_addr;
  Serial.print("\n📩 Received response:");
  Serial.println(strlen((const char *)incomingData));
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, incomingData, len);

  if (error)
  {
    Serial.print("deserializeJson() failed:");
    Serial.println(error.c_str());
    return;
  }
  processReceivedData(doc, mac_addr);
}

#endif // ESPNOW_HANDLER_H