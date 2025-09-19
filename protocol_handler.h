#ifndef PROTOCOL_HANDLER_H
#define PROTOCOL_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESP32_NOW.h>
#include <ArduinoJson.h>
// #include <deque>

#include "config.h"
#include "led_status.h"
#include "serial.h"
#include "function.h"

void addMacToList(int id, int lid, const uint8_t *mac_addr, unsigned long time_);

// Tạo tin nhắn phản hồi
String createMessage(int id_src, int id_des, String mac_src, String mac_des, uint8_t opcode, DynamicJsonDocument data, unsigned long timestamp = 0)
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
  String mac_src = message["mac_src"];
  String mac_des = message["mac_des"];
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
    Serial.println("OK!");
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
    Serial.println("Đã nhận phản hồi HUB_GET_LICENSE:");
    JsonObject data = message["data"];
    int lid = data["lid"];
    unsigned long time_temp = data["remain"];

    addMacToList(id_src, lid, mac_addr, time_temp);
    // printDeviceList();
    // Serial.println(data["license"].as<String>());
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
    Serial.println("❌ Payload quá lớn!");
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
  DynamicJsonDocument dataDoc(128);
  dataDoc["lid"] = lid;
  String output = createMessage(id_src, id_des, mac, mac_des, opcode, dataDoc, now);

  if (output.length() > sizeof(message.payload))
  {
    Serial.println("❌ Payload quá lớn!");
    return;
  }

  output.toCharArray(message.payload, sizeof(message.payload));
  esp_now_send(receiverMac, (uint8_t *)&message, sizeof(message));

  Serial.println("📤 Gửi HUB_GET_LICENSE:");
  Serial.println(output);
}

#endif
