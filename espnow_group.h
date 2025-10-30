#ifndef ESPNOW_GROUP_H
#define ESPNOW_GROUP_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
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
extern uint8_t configuredAssignmentMac[MAX_DEVICES][6];
extern uint8_t configuredAssignmentGroupId[MAX_DEVICES];

/// @brief Số nhóm mặc định mà Hub luôn cố gắng duy trì.
constexpr uint8_t DEFAULT_TARGET_GROUP_COUNT = 5;
/// @brief Giới hạn cứng số thiết bị trong một nhóm.
constexpr uint8_t MAX_DEVICES_PER_GROUP = 20;
/// @brief Thời gian chờ phản hồi mặc định cho mỗi nhóm (ms).
constexpr uint32_t DEFAULT_GROUP_RESPONSE_WINDOW_MS = 3000; // cũ 1500
/// @brief Ngưỡng tối thiểu để tránh cấu hình thời gian chờ quá thấp (ms).
constexpr uint32_t MIN_GROUP_RESPONSE_WINDOW_MS = 500;
/// @brief Số bản ghi gán nhóm tối đa được gửi kèm trong một gói JSON.
constexpr size_t MAX_GROUP_ASSIGNMENTS_IN_MESSAGE = 24;

inline bool isZeroMac(const uint8_t mac[6])
{
  if (mac == nullptr)
  {
    return true;
  }
  for (size_t i = 0; i < 6; i++)
  {
    if (mac[i] != 0)
    {
      return false;
    }
  }
  return true;
}

inline bool parseMacString(const char *text, uint8_t outMac[6])
{
  if (text == nullptr || outMac == nullptr)
  {
    return false;
  }

  int values[6] = {0};
  if (sscanf(text, "%x:%x:%x:%x:%x:%x",
             &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5]) == 6)
  {
    for (size_t i = 0; i < 6; i++)
    {
      if (values[i] < 0 || values[i] > 0xFF)
      {
        return false;
      }
      outMac[i] = static_cast<uint8_t>(values[i]);
    }
    return true;
  }

  size_t len = strlen(text);
  if (len == 12)
  {
    for (size_t i = 0; i < 6; i++)
    {
      char buf[3] = {text[i * 2], text[i * 2 + 1], '\0'};
      if (!isxdigit(buf[0]) || !isxdigit(buf[1]))
      {
        return false;
      }
      outMac[i] = static_cast<uint8_t>(strtoul(buf, nullptr, 16));
    }
    return true;
  }

  return false;
}

inline void clearConfiguredGroupAssignments()
{
  configuredAssignmentCount = 0;
  memset(configuredAssignmentMac, 0, sizeof(configuredAssignmentMac));
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
    memcpy(configuredAssignmentMac[i - 1], configuredAssignmentMac[i], 6);
    configuredAssignmentGroupId[i - 1] = configuredAssignmentGroupId[i];
  }

  memset(configuredAssignmentMac[configuredAssignmentCount - 1], 0, 6);
  configuredAssignmentGroupId[configuredAssignmentCount - 1] = 0;
  configuredAssignmentCount--;
}

inline void setConfiguredGroupAssignment(const uint8_t mac[6], uint8_t groupId)
{
  if (isZeroMac(mac))
  {
    return;
  }

  for (size_t i = 0; i < configuredAssignmentCount; i++)
  {
    if (memcmp(configuredAssignmentMac[i], mac, 6) == 0)
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

  memcpy(configuredAssignmentMac[configuredAssignmentCount], mac, 6);
  configuredAssignmentGroupId[configuredAssignmentCount] = groupId;
  configuredAssignmentCount++;
}

inline uint8_t getConfiguredGroupForMac(const uint8_t mac[6])
{
  if (isZeroMac(mac))
  {
    return 0;
  }

  for (size_t i = 0; i < configuredAssignmentCount; i++)
  {
    if (memcmp(configuredAssignmentMac[i], mac, 6) == 0)
    {
      return configuredAssignmentGroupId[i];
    }
  }
  return 0;
}

inline bool assignConfiguredGroupForLocalId(int localId, uint8_t groupId)
{
  if (localId <= 0)
  {
    return false;
  }

  bool assigned = false;
  for (int i = 0; i < Device.deviceCount; i++)
  {
    if (Device.LocalID[i] == localId)
    {
      setConfiguredGroupAssignment(Device.MACList[i], groupId);
      assigned = true;
    }
  }

  if (!assigned)
  {
    Serial.printf("⚠️ Không tìm thấy MAC cho Local ID %d khi cấu hình nhóm.\n", localId);
  }

  return assigned;
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
    bool handled = false;

    const char *macText = nullptr;
    if (obj.containsKey("mac"))
    {
      macText = obj["mac"].as<const char *>();
    }
    else if (obj.containsKey("mac_addr"))
    {
      macText = obj["mac_addr"].as<const char *>();
    }
    if (macText != nullptr)
    {
      uint8_t mac[6] = {0};
      if (parseMacString(macText, mac))
      {
        setConfiguredGroupAssignment(mac, groupId);
        handled = true;
      }
      else
      {
        Serial.printf("⚠️ Không thể phân tích địa chỉ MAC '%s' trong cấu hình nhóm.\n",
                      macText);
      }
    }

    if (obj.containsKey("value"))
    {
      parseGroupAssignmentVariant(obj["value"], groupId);
      handled = true;
    }

    if (obj.containsKey("lid"))
    {
      handled = assignConfiguredGroupForLocalId(obj["lid"].as<int>(), groupId) || handled;
    }
    if (obj.containsKey("local_id"))
    {
      handled = assignConfiguredGroupForLocalId(obj["local_id"].as<int>(), groupId) || handled;
    }
    if (obj.containsKey("id"))
    {
      handled = assignConfiguredGroupForLocalId(obj["id"].as<int>(), groupId) || handled;
    }

    if (!handled)
    {
      Serial.println("⚠️ Bỏ qua cấu hình nhóm không xác định (không có trường mac/lid).");
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

    uint8_t mac[6] = {0};
    if (parseMacString(text, mac))
    {
      setConfiguredGroupAssignment(mac, groupId);
      return;
    }

    const char *cursor = text;
    while (*cursor != '\0')
    {
      char *endPtr = nullptr;
      long value = strtol(cursor, &endPtr, 10);
      if (endPtr == cursor)
      {
        if (*cursor == '\0')
        {
          break;
        }
        cursor++;
        continue;
      }

      if (value != 0)
      {
        assignConfiguredGroupForLocalId(static_cast<int>(value), groupId);
      }
      cursor = endPtr;
    }
    return;
  }

  if (entry.is<int>() || entry.is<long>() || entry.is<unsigned int>() || entry.is<unsigned long>())
  {
    assignConfiguredGroupForLocalId(entry.as<int>(), groupId);
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
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             Device.MACList[i][0],
             Device.MACList[i][1],
             Device.MACList[i][2],
             Device.MACList[i][3],
             Device.MACList[i][4],
             Device.MACList[i][5]);
    entry["mac"] = macStr;
    entry["lid"] = Device.LocalID[i];
    entry["gid"] = ensureDeviceGroup(i);
  }
}

#endif // ESPNOW_GROUP_H