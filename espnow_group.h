#ifndef ESPNOW_GROUP_H
#define ESPNOW_GROUP_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>

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

/// @brief Thời gian chờ phản hồi mặc định cho mỗi nhóm (ms).
constexpr uint32_t DEFAULT_GROUP_RESPONSE_WINDOW_MS = 3000; // cũ 1500
/// @brief Ngưỡng tối thiểu để tránh cấu hình thời gian chờ quá thấp (ms).
constexpr uint32_t MIN_GROUP_RESPONSE_WINDOW_MS = 500;
/// @brief Chuẩn hóa số lượng nhóm được yêu cầu, đảm bảo tối thiểu là 1.
inline uint8_t resolveRequestedGroupCount()
{
  return desired_group_count == 0 ? 1 : desired_group_count;
}

/// @brief Chuẩn hóa kích thước nhóm theo cấu hình và đảm bảo giá trị hợp lệ.
inline uint8_t resolveRequestedGroupSize()
{
  uint8_t size = desired_group_size;
  if (size == 0)
  {
    size = 1;
  }
  return size;
}

/**
 * @brief Xác định group id cần sử dụng cho thiết bị ở index cho trước.
 *
 * Hàm ánh xạ vị trí của thiết bị trong danh sách sang group dựa trên kích thước
 * nhóm hiện hành để đảm bảo các nhóm được lấp đầy tuần tự và đồng đều.
 */
inline uint8_t resolveGroupIdForIndex(int index)
{
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
    // total = DEFAULT_TARGET_GROUP_COUNT;
    total = static_cast<uint32_t>(resolveRequestedGroupCount()) * resolveRequestedGroupSize();
  }
  return total;
}

/**
 * @brief Đồng bộ lại cấu hình nhóm theo các giá trị hiện tại của hệ thống.
 *
 * Hàm này chịu trách nhiệm giữ cho groupSize và groupCount luôn phản ánh mục
 * tiêu được xác định qua desired_group_count/desired_group_size. Nó cũng đảm
 * bảo responseWindowMs nằm trong giới hạn cho phép để tránh cấu hình lỗi.
 *
 */

inline void applyConfiguredGroupsToKnownDevices();

inline void refreshGroupConfiguration()
{
  groupConfig.totalNodes = getPlannedNodeCount();

  uint32_t targetCount = resolveRequestedGroupCount();
  if (targetCount == 0)
  {
    targetCount = 1;
  }

  uint32_t effectiveNodes = groupConfig.totalNodes;
  uint8_t requestedGroupSize = resolveRequestedGroupSize();
  uint32_t maxSupportedNodes = static_cast<uint32_t>(requestedGroupSize) * targetCount;
  if (effectiveNodes > maxSupportedNodes)
  {
    effectiveNodes = maxSupportedNodes;
  }

  uint32_t desiredSize32 = (effectiveNodes + targetCount - 1) / targetCount;
  if (desiredSize32 == 0)
  {
    desiredSize32 = 1;
  }
  if (desiredSize32 > requestedGroupSize)
  {
    desiredSize32 = requestedGroupSize;
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
    Device.groupId[i] = resolveGroupIdForIndex(i);
  }
}

inline void recalcAndApplyGroupConfiguration()
{
  refreshGroupConfiguration();
  applyConfiguredGroupsToKnownDevices();
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

  if (Device.groupId[index] == 0 || Device.groupId[index] > groupConfig.groupCount)
  {
    Device.groupId[index] = resolveGroupIdForIndex(index);
  }
  return Device.groupId[index];
}

/**
 * @brief Tìm index trong danh sách thiết bị dựa trên địa chỉ MAC.
 */
inline int findDeviceIndexByMac(const uint8_t mac[6])
{
  if (mac == nullptr)
  {
    return -1;
  }

  for (int i = 0; i < Device.deviceCount; i++)
  {
    if (memcmp(Device.MACList[i], mac, 6) == 0)
    {
      return i;
    }
  }
  return -1;
}

/**
 * @brief Đảm bảo và trả về group id gắn với thiết bị có MAC tương ứng.
 */
inline uint8_t ensureDeviceGroupForMac(const uint8_t mac[6])
{
  int index = findDeviceIndexByMac(mac);
  if (index < 0)
  {
    return 0;
  }
  return ensureDeviceGroup(index);
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

/// @brief Tìm group id nhỏ nhất còn node đang chờ phản hồi.
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
    // lowest = 1;
    // Không có node nào đang pending nên không nên ép Hub gửi group_id=1.
    return 0;
  }
  return lowest;
}

/**
 * @brief Gửi kèm thông tin cấu hình nhóm vào JSON trả về cho thiết bị.
 *
 * Payload luôn chứa metadata tối thiểu (count/size) để node tự điều chỉnh thứ tự
 * phản hồi. Tham số includeAssignments được giữ lại nhằm tương thích với API
 * cũ nhưng hiện không còn thêm danh sách gán chi tiết.
 */
inline void appendGroupConfiguration(DynamicJsonDocument &dataDoc, uint8_t targetGroupId, bool includeAssignments)
{
  JsonObject groupCfg = dataDoc.createNestedObject("group_cfg");
  if (targetGroupId != 0)
  {
    groupCfg["target_group_id"] = targetGroupId;
  }

  groupCfg["count"] = groupConfig.groupCount;
  groupCfg["size"] = groupConfig.groupSize;

  if (!includeAssignments)
  {
    return;
  }

  JsonArray assignments = groupCfg.createNestedArray("assignments");
  for (int i = 0; i < Device.deviceCount; i++)
  {
    uint8_t deviceGroup = ensureDeviceGroup(i);
    if (deviceGroup == 0)
    {
      continue;
    }

    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             Device.MACList[i][0], Device.MACList[i][1], Device.MACList[i][2],
             Device.MACList[i][3], Device.MACList[i][4], Device.MACList[i][5]);

    JsonObject entry = assignments.createNestedObject();
    entry["mac"] = macStr;
    entry["group_id"] = deviceGroup;
  }
}

#endif // ESPNOW_GROUP_H