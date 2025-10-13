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

// void processReceivedData(StaticJsonDocument<512> &message, const uint8_t *mac_addr);

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

void addMacToList(int id, int lid, const uint8_t *mac_addr, unsigned long time_, uint8_t groupId)
{
  // Kiểm tra xem node đã có trong danh sách hay chưa
  int existingIndex = findMacIndex(mac_addr);

  if (existingIndex != -1)
  {
    Device.DeviceID[existingIndex] = id;
    Device.LocalID[existingIndex] = lid;
    Device.timeLIC[existingIndex] = time_;
    // Cập nhật thống kê phản hồi khi thiết bị trả lời
    Device.responseCount[existingIndex]++;
    Device.lastResponseMillis[existingIndex] = millis();
    Device.pendingResponse[existingIndex] = false;
    Device.groupId[existingIndex] = resolveGroupIdForIndex(existingIndex, groupId);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print("Thiết bị đã tồn tại: ");
    Serial.print(macStr);
    Serial.print(" -Nhóm:");
    Serial.print(Device.groupId[existingIndex]);
    Serial.print(" -số lần phản hồi:");
    Serial.println(Device.responseCount[existingIndex]);
    if (groupId == 0)
    {
      Serial.println("⚠️ Node không gửi group_id, áp dụng nhóm theo cấu hình hiện tại.");
    }
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
  Device.responseCount[index] = 1;              // Lần phản hồi đầu tiên
  Device.lastResponseMillis[index] = millis();  // Ghi nhận thời điểm phản hồi
  Device.lastRequestMillis[index] = 0;          // Chưa có lần yêu cầu đơn lẻ nào
  Device.pendingResponse[index] = false;        // Đã phản hồi cho lượt broadcast hiện tại
  Device.groupId[index] = resolveGroupIdForIndex(index, groupId);

  Device.deviceCount++;
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Thiết bị mới:");
  Serial.println(macStr);
  Serial.print("Nhóm: ");
  Serial.println(Device.groupId[index]);
  Serial.print("Số lần phản hồi: ");
  Serial.println(Device.responseCount[index]);
  if (groupId == 0)
  {
    Serial.println("⚠️ Node không gửi group_id, áp dụng nhóm theo cấu hình hiện tại.");
  }
}

void printDeviceList()
{
  Serial.println("Danh sách thiết bị đã tìm thấy:");
  for (int i = 0; i < Device.deviceCount; i++)
  {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             Device.MACList[i][0], Device.MACList[i][1], Device.MACList[i][2],
             Device.MACList[i][3], Device.MACList[i][4], Device.MACList[i][5]);
    Serial.print("Thiết bị ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(macStr);
    Serial.print("  Số lần phản hồi: ");
    Serial.println(Device.responseCount[i]);
    Serial.print("Nhóm:");
    Serial.println(Device.groupId[i]);
  }
  Serial.println("------------------");
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Send Status: ");

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? " Success" : " Fail");
}


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

// void addPeer(uint8_t *macAddr)
// {
//   esp_now_peer_info_t peerInfo = {};
//   memcpy(peerInfo.peer_addr, macAddr, 6); // FF:FF:FF:FF
//   peerInfo.channel = 1;                   // Kênh cố định để đồng bộ với sender
//   peerInfo.encrypt = false;               // Tạm thời tắt mã hóa
//   if (esp_now_add_peer(&peerInfo) != ESP_OK)
//   {
//     Serial.println("❌ Failed to add peer!");
//   }
//   else
//   {
//     Serial.println("add peer ok");
//   }
// }

#endif // ESPNOW_HANDLER_H
