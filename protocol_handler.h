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
  if(targetGroupId !=0)
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

void config_device(int id_des, int lid, String mac_des, uint32_t nod, unsigned long now)
{
  int opcode = LIC_CONFIG_DEVICE;
  String mac = WiFi.macAddress();
  int id_src = config_id;
  DynamicJsonDocument dataDoc(256);
  dataDoc["id"] = id_des;
  dataDoc["lid"] = lid;
  dataDoc["nod"] = nod;
  appendGroupConfiguration(dataDoc, 0, false);

  String output = createMessage(id_src, id_des, mac, mac_des, opcode, dataDoc, now);

  if (output.length() > sizeof(message.payload))
  {
    Serial.println("❌ Payload quá lớn! 3");
    return;
  }

  output.toCharArray(message.payload, sizeof(message.payload));
  esp_now_send(receiverMac, (uint8_t *)&message, sizeof(message));

  Serial.println("📤 Gửi LIC_CONFIG_DEVICE:");
  Serial.println(output);
}
#endif
