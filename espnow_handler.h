#ifndef ESPNOW_HANDLER_H
#define ESPNOW_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include "ESP32_NOW.h"
#include <ArduinoJson.h>
#include <cstring>

#include "config.h"
#include "protocol_handler.h"

// void processReceivedData(StaticJsonDocument<512> &message, const uint8_t *mac_addr);

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

bool isMacExist(const uint8_t *mac_addr)
{
  return findMacIndex(mac_addr) != -1;
}

void addMacToList(int id, int lid, const uint8_t *mac_addr, unsigned long time_)
{
  int existingIndex = findMacIndex(mac_addr);

  if (existingIndex != -1)
  {
    Device.DeviceID[existingIndex] = id;
    Device.LocalID[existingIndex] = lid;
    Device.timeLIC[existingIndex] = time_;
    Device.responseCount[existingIndex]++;

    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print("Thiết bị đã tồn tại: ");
    Serial.print(macStr);
    Serial.print(" - số lần phản hồi: ");
    Serial.println(Device.responseCount[existingIndex]);
    return;
  }

  if (Device.deviceCount >= MAX_DEVICES)
  {
    Serial.println("Không thể thêm thiết bị mới: đã đạt giới hạn.");
    return;
  }

  int index = Device.deviceCount;
  memcpy(Device.MACList[index], mac_addr, 6);

  Device.DeviceID[index] = id;
  Device.LocalID[index] = lid;
  Device.timeLIC[index] = time_;
  Device.responseCount[index] = 1;

  Device.deviceCount++;
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Thiết bị mới: ");
  Serial.println(macStr);
  Serial.print("Số lần phản hồi: ");
  Serial.println(Device.responseCount[index]);
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
  }
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
  }
  Serial.println("------------------");
}

//-----------------------------------------------

// Kiểm tra xem địa chỉ MAC đã tồn tại trong danh sách chưa

// bool isMacExist(const uint8_t *mac_addr)
// {
//   for (int i = 0; i < Device.deviceCount; i++)
//   {
//     if (memcmp(Device.MACList[i], mac_addr, 6) == 0)
//     {
//       return true; // MAC đã tồn tại
//     }
//   }
//   return false;
// }

// Thêm Mac vào danh sách
//  void addMacToList(int id, int lid, const uint8_t *mac_addr, unsigned long time_)
//  {
//    if (Device.deviceCount < MAX_DEVICES && !isMacExist(mac_addr))
//    {
//      memcpy(Device.MACList[Device.deviceCount], mac_addr, 6);

//     Device.DeviceID[Device.deviceCount] = id;
//     Device.LocalID[Device.deviceCount] = lid;
//     Device.timeLIC[Device.deviceCount] = time_;

//     Device.deviceCount++;
//     char macStr[18];
//     snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
//              mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//     Serial.print("Thiết bị mới: ");
//     Serial.println(macStr);
//   }
// }

// Hiển thị danh sách thiết bị đã lưu
//  void printDeviceList()
//  {
//    Serial.println("Danh sách thiết bị đã tìm thấy:");
//    for (int i = 0; i < Device.deviceCount; i++)
//    {
//      char macStr[18];
//      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
//               Device.MACList[i][0], Device.MACList[i][1], Device.MACList[i][2],
//               Device.MACList[i][3], Device.MACList[i][4], Device.MACList[i][5]);
//      Serial.print("Thiết bị ");
//      Serial.print(i + 1);
//      Serial.print(": ");
//      Serial.println(macStr);
//    }
//    Serial.println("------------------");
//  }

// Xử lý khi nhận phản hồi
//  void handleScanResponse(const uint8_t *mac, int node_id, int local_id)
//  {
//    if (!isMacExist(mac))
//    {
//      addMacToList(node_id, local_id, mac, millis());
//    }
//    else
//    {
//      Serial.println("Thiết bị đã tồn tại, bỏ qua phản hồi");
//    }
//  }

void print_mac(const uint8_t *mac)
{
  // Hàm in địa chỉ MAC từ một mảng 6 byte
  for (int i = 0; i < 6; i++)
  {
    if (mac[i] < 0x10)
    {
      Serial.print("0"); // thêm số 0 nếu byte < 16 (để đảm bảo 2 chữ số)
    }
    Serial.print(mac[i], HEX); // In byte dưới dạng thập lục phân
    if (i < 5)
    {
      Serial.print(":"); // Thêm dấu: giữa các byte, trừ byte cuối
    }
  }
  Serial.println(); // Xuống dòng sau khi in
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
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }
  processReceivedData(doc, mac_addr);
}

void addPeer(uint8_t *macAddr)
{
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, macAddr, 6); // FF:FF:FF:FF
  peerInfo.channel = 1;                   // Kênh cố định để đồng bộ với sender
  peerInfo.encrypt = false;               // Tạm thời tắt mã hóa
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("❌ Failed to add peer!");
  }
  else
  {
    Serial.println("add peer ok");
  }
}

#endif // ESPNOW_HANDLER_H



