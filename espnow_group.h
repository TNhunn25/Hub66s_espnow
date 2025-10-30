#ifndef ESPNOW_GROUP_H
#define ESPNOW_GROUP_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "function.h"
#include "config.h"

/**
 * @brief Cấu hình nhóm dùng để điều phối phản hồi giữa các node ESPNOW.
 */
typedef struct
{
  uint8_t groupSize;         ///< Số thiết bị tối đa trong một nhóm phản hồi.
  uint8_t groupCount;        ///< Tổng số nhóm đang hoạt động.
  uint32_t totalNodes;       ///< Tổng số node dự kiến tham gia phản hồi.
  uint32_t responseWindowMs; ///< Thời gian chờ phản hồi tối đa cho mỗi nhóm.
} group_config_t;

extern group_config_t groupConfig;
extern size_t configuredAssignmentCount;
extern uint8_t configuredAssignmentMacList[MAX_DEVICES][6];
extern uint8_t configuredAssignmentGroupId[MAX_DEVICES];

/// @brief Số nhóm mặc định mà Hub luôn cố gắng duy trì.
constexpr uint8_t DEFAULT_TARGET_GROUP_COUNT = 5;
/// @brief Giới hạn cứng số thiết bị trong một nhóm.
constexpr uint8_t MAX_DEVICES_PER_GROUP = 20;
/// @brief Thời gian chờ phản hồi mặc định cho mỗi nhóm (ms).
constexpr uint32_t DEFAULT_GROUP_RESPONSE_WINDOW_MS = 3000; // cũ 1500
constexpr uint32_t MIN_GROUP_RESPONSE_WINDOW_MS = 500;
/// @brief Số bản ghi gán nhóm tối đa được gửi kèm trong một gói JSON.
constexpr size_t MAX_GROUP_ASSIGNMENTS_IN_MESSAGE = 24;
constexpr size_t MAC_ADDRESS_LENGTH = 6;

inline bool macIsEmpty(const uint8_t mac[6])
{
  static const uint8_t ZERO_MAC[MAC_ADDRESS_LENGTH] = {0};
  return memcmp(mac, ZERO_MAC, MAC_ADDRESS_LENGTH) == 0;
}

inline bool macEquals(const uint8_t lhs[6], const uint8_t rhs[6])
{
  return memcmp(lhs, rhs, MAC_ADDRESS_LENGTH) == 0;
}

inline void copyMac(uint8_t dest[6], const uint8_t src[6])
{
  memcpy(dest, src, MAC_ADDRESS_LENGTH);
}

inline bool parseMacString(const char *text, uint8_t outMac[6])
{
  if (text == nullptr)
  {
    return false;
  }

  int values[MAC_ADDRESS_LENGTH];
  if (sscanf(text, "%x:%x:%x:%x:%x:%x",
             &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]) != 6)
  {
    return false;
  }

  for (int i = 0; i < static_cast<int>(MAC_ADDRESS_LENGTH); i++)
  {
    if (values[i] < 0 || values[i] > 0xFF)
    {
      return false;
    }
    outMac[i] = static_cast<uint8_t>(values[i]);
  }
  return true;
}

inline bool resolveUniqueMacForDeviceId(int deviceId, uint8_t outMac[6])
{
  if (deviceId <= 0)
  {
    return false;
  }

  int matchedIndex = -1;
  for (int i = 0; i < Device.deviceCount; i++)
  {
    if (Device.DeviceID[i] != deviceId)
    {
      continue;
    }

    if (matchedIndex == -1)
    {
      matchedIndex = i;
      copyMac(outMac, Device.MACList[i]);
    }
    else if (!macEquals(Device.MACList[i], Device.MACList[matchedIndex]))
    {
      Serial.printf("⚠️ Bỏ qua cấu hình nhóm vì DeviceID %d trùng lặp và ánh xạ nhiều MAC khác nhau.\n", deviceId);
      return false;
    }
  }

  if (matchedIndex == -1)
  {
    Serial.printf("⚠️ Bỏ qua cấu hình nhóm vì không tìm thấy DeviceID %d trong danh sách thiết bị hiện tại.\n", deviceId);
    return false;
  }

  return true;
}

inline bool resolveUniqueMacForLocalId(int localId, uint8_t outMac[6])
{
  if (localId <= 0)
  {
    return false;
  }

  int matchedIndex = -1;
  for (int i = 0; i < Device.deviceCount; i++)
  {
    if (Device.LocalID[i] != localId)
    {
      continue;
    }

    if (matchedIndex == -1)
    {
      matchedIndex = i;
      copyMac(outMac, Device.MACList[i]);
    }
    else if (!macEquals(Device.MACList[i], Device.MACList[matchedIndex]))
    {
      Serial.printf("⚠️ Bỏ qua cấu hình nhóm vì LocalID %d ánh xạ tới nhiều MAC khác nhau. Vui lòng cấu hình theo MAC.\n", localId);
      return false;
    }
  }

  if (matchedIndex == -1)
  {
    Serial.printf("⚠️ Bỏ qua cấu hình nhóm vì không tìm thấy LocalID %d trong danh sách thiết bị hiện tại.\n", localId);
    return false;
  }

  return true;
}

inline void clearConfiguredGroupAssignments()
{
  configuredAssignmentCount = 0;
  memset(configuredAssignmentMacList, 0, sizeof(configuredAssignmentMacList));
  memset(configuredAssignmentGroupId, 0, sizeof(configuredAssignmentGroupId));
}

inline void removeConfiguredGroupAssignmentAt(size_t idx)
{
  if (idx >= configuredAssignmentCount)
  {
    return;
  }

  for (size_t i = idx + 1; i < configuredAssignmentCount; i++)
  {
    copyMac(configuredAssignmentMacList[i - 1], configuredAssignmentMacList[i]);
    configuredAssignmentGroupId[i - 1] = configuredAssignmentGroupId[i];
  }

  memset(configuredAssignmentMacList[configuredAssignmentCount - 1], 0, MAC_ADDRESS_LENGTH);
  configuredAssignmentGroupId[configuredAssignmentCount - 1] = 0;
  configuredAssignmentCount--;
}

inline void setConfiguredGroupAssignmentForMac(const uint8_t mac[6], uint8_t groupId)
{
  if (mac == nullptr || macIsEmpty(mac))
  {
    return;
  }

  for (size_t i = 0; i < configuredAssignmentCount; i++)
  {
    if (macEquals(configuredAssignmentMacList[i], mac))
    {
      if (groupId == 0)
      {
        removeConfiguredGroupAssignmentAt(i);
      }
      else
      {
        configuredAssignmentGroupId[i] = groupId;
      }
      return;
    }
  }

  if (groupId == 0 || configuredAssignmentCount >= MAX_DEVICES)
  {
    return;
  }

  copyMac(configuredAssignmentMacList[configuredAssignmentCount], mac);
  configuredAssignmentGroupId[configuredAssignmentCount] = groupId;
  configuredAssignmentCount++;
}

inline uint8_t getConfiguredGroupForMac(const uint8_t mac[6])
{
  if (mac == nullptr || macIsEmpty(mac))
  {
    return 0;
  }
  for (size_t i = 0; i < configuredAssignmentCount; i++)
  {
    if (macEquals(configuredAssignmentMacList[i], mac))
    {
      return configuredAssignmentGroupId[i];
    }
  }
  return 0;
}

inline uint8_t getHighestConfiguredGroupId()
{
  uint8_t highest = 0;
  for (size_t i = 0; i < configuredAssignmentCount; i++)
  {
    if (configuredAssignmentGroupId[i] > highest)
    {
      highest = configuredAssignmentGroupId[i];
    }
  }
  return highest;
}

/**
 * @brief Xác định group id cần sử dụng cho thiết bị ở index cho trước.
 *
 * Khi firmware hoặc bản cấu hình yêu cầu một group cụ thể (requestedGroup),
 * giá trị đó sẽ được trả về ngay. Ngược lại, hàm sẽ ánh xạ index của thiết bị
 * sang group dựa trên groupSize hiện hành để đảm bảo các nhóm được lấp đầy
 * tuần tự.
 */
inline uint8_t resolveGroupIdForIndex(int index, uint8_t requestedGroup)
{
  if (requestedGroup != 0)
  {
    return requestedGroup;
  }

  uint8_t size = groupConfig.groupSize;
  if (size == 0)
  {
    size = 1;
  }

  uint8_t groupId = static_cast<uint8_t>((index / size) + 1);
  if (groupId > groupConfig.groupCount && groupConfig.groupCount != 0)
  {
    groupId = groupConfig.groupCount;
  }
  return groupId;
}

/**
 * @brief Ước lượng số node sẽ tham gia, ưu tiên số lượng thiết bị đã biết.
 *
 * Hub lấy số lượng thiết bị đã lưu (Device.deviceCount) và giá trị cấu hình
 * yêu cầu (nod), sau đó chọn giá trị lớn hơn để đảm bảo kích thước nhóm được
 * tính toán đủ rộng.
 */
inline uint32_t getPlannedNodeCount()
{
  uint32_t knownDevices = Device.deviceCount;
  uint32_t configuredNodes = nod;
  uint32_t total = knownDevices > configuredNodes ? knownDevices : configuredNodes;
  if (total == 0)
  {
    total = DEFAULT_TARGET_GROUP_COUNT;
  }
  return total;
}

/**
 * @brief Đồng bộ lại cấu hình nhóm theo các giá trị hiện tại của hệ thống.
 *
 * Hàm này chịu trách nhiệm giữ cho groupSize và groupCount luôn phản ánh mục
 * tiêu "5 nhóm, tối đa 20 thiết bị". Nó cũng đảm bảo responseWindowMs nằm
 * trong giới hạn cho phép để tránh cấu hình lỗi.
 */
inline void refreshGroupConfiguration()
{
  groupConfig.totalNodes = getPlannedNodeCount();

  uint32_t targetCount = DEFAULT_TARGET_GROUP_COUNT;
  if (targetCount == 0)
  {
    targetCount = 1;
  }

  uint32_t effectiveNodes = groupConfig.totalNodes;
  uint32_t maxSupportedNodes = static_cast<uint32_t>(MAX_DEVICES_PER_GROUP) * targetCount;
  if (effectiveNodes > maxSupportedNodes)
  {
    effectiveNodes = maxSupportedNodes;
  }

  uint32_t desiredSize32 = (effectiveNodes + targetCount - 1) / targetCount;
  if (desiredSize32 == 0)
  {
    desiredSize32 = 1;
  }
  if (desiredSize32 > MAX_DEVICES_PER_GROUP)
  {
    desiredSize32 = MAX_DEVICES_PER_GROUP;
  }

  groupConfig.groupSize = static_cast<uint8_t>(desiredSize32);
  groupConfig.groupCount = static_cast<uint8_t>(targetCount);

  if (groupConfig.responseWindowMs == 0)
  {
    groupConfig.responseWindowMs = DEFAULT_GROUP_RESPONSE_WINDOW_MS;
  }
  if (groupConfig.responseWindowMs < MIN_GROUP_RESPONSE_WINDOW_MS)
  {
    groupConfig.responseWindowMs = MIN_GROUP_RESPONSE_WINDOW_MS;
  }
}
inline void applyConfiguredGroupsToKnownDevices()
{
  for (int i = 0; i < Device.deviceCount; i++)
  {
    uint8_t configuredGroup = getConfiguredGroupForMac(Device.MACList[i]);
    if (configuredGroup != 0)
    {
      Device.groupId[i] = configuredGroup;
    }
    else
    {
      Device.groupId[i] = resolveGroupIdForIndex(i, 0);
    }
  }
}

/**
 * @brief Đảm bảo thiết bị tại index đã được gán group id hợp lệ.
 *
 * Nếu chưa có group id, hàm sẽ tính toán và gán dựa trên vị trí thiết bị trong
 * danh sách. Điều này giúp việc truy vấn trạng thái nhóm nhanh chóng và thống
 * nhất.
 */
inline uint8_t ensureDeviceGroup(int index)
{
  if (index < 0 || index >= Device.deviceCount)
  {
    return 1;
  }
  uint8_t configuredGroup = getConfiguredGroupForMac(Device.MACList[index]);
  if (configuredGroup != 0)
  {
    Device.groupId[index] = configuredGroup;
    return configuredGroup;
  }

  if (Device.groupId[index] == 0)
  {
    Device.groupId[index] = resolveGroupIdForIndex(index, 0);
  }
  return Device.groupId[index];
}

/**
 * @brief Kiểm tra xem còn thiết bị nào trong group đang cần phản hồi hay không.
 */
inline bool hasPendingResponsesInGroup(uint8_t groupId)
{
  for (int i = 0; i < Device.deviceCount; i++)
  {
    if (!Device.pendingResponse[i])
    {
      continue;
    }

    uint8_t deviceGroup = ensureDeviceGroup(i);
    if (groupId == 0 || deviceGroup == groupId)
    {
      return true;
    }
  }
  return false;
}

inline void parseGroupAssignmentVariant(const JsonVariantConst &entry, uint8_t groupId)
{
  if (groupId == 0)
  {
    return;
  }

  if (entry.is<JsonArrayConst>())
  {
    for (JsonVariantConst nested : entry.as<JsonArrayConst>())
    {
      parseGroupAssignmentVariant(nested, groupId);
    }
    return;
  }

  if (entry.is<JsonObjectConst>())
  {
    JsonObjectConst obj = entry.as<JsonObjectConst>();
    if (obj.containsKey("value"))
    {
      parseGroupAssignmentVariant(obj["value"], groupId);
      return;
    }

    uint8_t mac[6] = {0};
    bool hasMac = false;

    if (obj.containsKey("mac"))
    {
      const char *macStr = obj["mac"].as<const char *>();
      hasMac = parseMacString(macStr, mac);
      if (!hasMac)
      {
        Serial.printf("⚠️ Bỏ qua cấu hình nhóm vì chuỗi MAC '%s' không hợp lệ.\n", macStr == nullptr ? "(null)" : macStr);
      }
    }
    else if (obj.containsKey("mac_addr"))
    {
      const char *macStr = obj["mac_addr"].as<const char *>();
      hasMac = parseMacString(macStr, mac);
      if (!hasMac)
      {
        Serial.printf("⚠️ Bỏ qua cấu hình nhóm vì mac_addr '%s' không hợp lệ.\n", macStr == nullptr ? "(null)" : macStr);
      }
    }
    else if (obj.containsKey("id"))
    {
      int deviceId = obj["id"].as<int>();
      hasMac = resolveUniqueMacForDeviceId(deviceId, mac);
    }
    else if (obj.containsKey("device_id"))
    {
      int deviceId = obj["device_id"].as<int>();
      hasMac = resolveUniqueMacForDeviceId(deviceId, mac);
    }
    else if (obj.containsKey("lid"))
    {
      int localId = obj["lid"].as<int>();
      hasMac = resolveUniqueMacForLocalId(localId, mac);
    }
    else if (obj.containsKey("local_id"))
    {
      int localId = obj["local_id"].as<int>();
      hasMac = resolveUniqueMacForLocalId(localId, mac);
    }

    if (hasMac)
    {
      setConfiguredGroupAssignmentForMac(mac, groupId);
    }
    return;
  }

  if (entry.is<const char *>())
  {
    const char *text = entry.as<const char *>();
    if (text == nullptr)
    {
      return;
    }

    const char *cursor = text;
    while (*cursor != '\0')
    {
      while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r' || *cursor == ',' || *cursor == ';')
      {
        cursor++;
      }
      if (*cursor == '\0')
      {
        break;
      }

      const char *start = cursor;
      while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' && *cursor != '\r' && *cursor != ',' && *cursor != ';')
      {
        cursor++;
      }

      size_t len = static_cast<size_t>(cursor - start);
      if (len == 0)
      {
        continue;
      }

      char token[32];
      if (len >= sizeof(token))
      {
        len = sizeof(token) - 1;
      }
      memcpy(token, start, len);
      token[len] = '\0';

      uint8_t mac[6] = {0};
      if (parseMacString(token, mac))
      {
        setConfiguredGroupAssignmentForMac(mac, groupId);
        continue;
      }

      long value = strtol(token, nullptr, 10);
      if (value != 0 && resolveUniqueMacForDeviceId(static_cast<int>(value), mac))
      {
        setConfiguredGroupAssignmentForMac(mac, groupId);
      }
    }
    return;
  }

  if (entry.is<int>() || entry.is<long>() || entry.is<unsigned int>() || entry.is<unsigned long>())
  {
    uint8_t mac[6] = {0};
    if (resolveUniqueMacForDeviceId(entry.as<int>(), mac))
    {
      setConfiguredGroupAssignmentForMac(mac, groupId);
    }
  }
}

inline void updateConfiguredGroupAssignmentsFromJson(JsonVariantConst groupVariant)
{
  if (groupVariant.isNull())
  {
    return;
  }

  if (!groupVariant.is<JsonObjectConst>())
  {
    Serial.println("⚠️ Trường Group không hợp lệ (không phải object).");
    return;
  }

  clearConfiguredGroupAssignments();

  JsonObjectConst groupObj = groupVariant.as<JsonObjectConst>();
  for (JsonPairConst kv : groupObj)
  {
    const char *keyStr = kv.key().c_str();
    if (keyStr == nullptr)
    {
      continue;
    }

    long groupValue = strtol(keyStr, nullptr, 10);
    if (groupValue <= 0 || groupValue > 255)
    {
      continue;
    }

    uint8_t groupId = static_cast<uint8_t>(groupValue);
    parseGroupAssignmentVariant(kv.value(), groupId);
  }

  refreshGroupConfiguration();
  applyConfiguredGroupsToKnownDevices();

  Serial.printf("⚙️ Đã cập nhật %u cấu hình nhóm, nhóm cao nhất: %u.\n",
                static_cast<unsigned int>(configuredAssignmentCount),
                getHighestConfiguredGroupId());
}

inline uint8_t findLowestPendingGroupId()
{
  uint8_t lowest = 0;
  for (int i = 0; i < Device.deviceCount; i++)
  {
    if (!Device.pendingResponse[i])
    {
      continue;
    }

    uint8_t deviceGroup = ensureDeviceGroup(i);
    if (deviceGroup == 0)
    {
      continue;
    }

    if (lowest == 0 || deviceGroup < lowest)
    {
      lowest = deviceGroup;
    }
  }

  if (lowest == 0 && groupConfig.groupCount > 0)
  {
    lowest = 1;
  }
  return lowest;
}

/**
 * @brief Gửi kèm thông tin cấu hình nhóm vào JSON trả về cho thiết bị.
 *
 * Payload chỉ giữ lại danh sách gán Local ID -> Group ID để node biết mình
 * thuộc nhóm nào và phản hồi theo thứ tự nhóm (1, 2, 3, ...). Các thông số
 * như group_count, group_size hay range được loại bỏ nhằm rút gọn bản tin.
 *
 * Khi includeAssignments = false, hàm sẽ không thêm trường group_cfg để tránh
 * gửi kèm metadata nhóm trong payload.
 */
inline bool hasLocalIdAssigned(const int *localIds, size_t count, int localId)
{
  for (size_t i = 0; i < count; i++)
  {
    if (localIds[i] == localId)
    {
      return true;
    }
  }
  return false;
}

inline void appendGroupConfiguration(DynamicJsonDocument &dataDoc, uint8_t targetGroupId, bool includeAssignments)
{
  if (!includeAssignments)
  {
    return;
  }

  JsonObject groupCfg = dataDoc.createNestedObject("group_cfg");
  if (targetGroupId != 0)
  {
    groupCfg["target_group_id"] = targetGroupId;
  }

  JsonArray assignments = groupCfg.createNestedArray("assignments");
  size_t includedAssignments = Device.deviceCount;
  if (includedAssignments > MAX_GROUP_ASSIGNMENTS_IN_MESSAGE)
  {
    includedAssignments = MAX_GROUP_ASSIGNMENTS_IN_MESSAGE;
  }

  for (size_t i = 0; i < includedAssignments; i++)
  {
    JsonObject entry = assignments.createNestedObject();
    entry["id"] = Device.DeviceID[i];
    entry["lid"] = Device.LocalID[i];
    entry["gid"] = ensureDeviceGroup(i);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             Device.MACList[i][0], Device.MACList[i][1], Device.MACList[i][2],
             Device.MACList[i][3], Device.MACList[i][4], Device.MACList[i][5]);
    entry["mac"] = macStr;
  }
}

#endif // ESPNOW_GROUP_H