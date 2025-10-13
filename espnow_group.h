// #ifndef ESPNOW_GROUP_H
// #define ESPNOW_GROUP_H

// #include <Arduino.h>
// #include <ArduinoJson.h>

// #include "function.h"
// #include "config.h"

// /**
//  * @brief Cấu hình nhóm dùng để điều phối phản hồi giữa các node ESPNOW.
//  */
// typedef struct
// {
//   uint8_t groupSize;         ///< Số thiết bị tối đa trong một nhóm phản hồi.
//   uint8_t groupCount;        ///< Tổng số nhóm đang hoạt động.
//   uint32_t totalNodes;       ///< Tổng số node dự kiến tham gia phản hồi.
//   uint32_t responseWindowMs; ///< Thời gian chờ phản hồi tối đa cho mỗi nhóm.
// } group_config_t;

// extern group_config_t groupConfig;

// constexpr uint8_t DEFAULT_TARGET_GROUP_COUNT = 4;
// constexpr uint32_t DEFAULT_GROUP_RESPONSE_WINDOW_MS = 1500;
// constexpr uint32_t MIN_GROUP_RESPONSE_WINDOW_MS = 500;
// constexpr size_t MAX_GROUP_ASSIGNMENTS_IN_MESSAGE = 24;

// inline uint8_t resolveGroupIdForIndex(int index, uint8_t requestedGroup)
// {
//   if (requestedGroup != 0)
//   {
//     return requestedGroup;
//   }

//   uint8_t size = groupConfig.groupSize;
//   if (size == 0)
//   {
//     size = 1;
//   }

//   return (index / size) + 1;
// }

// inline uint32_t getPlannedNodeCount()
// {
//   uint32_t knownDevices = Device.deviceCount;
//   uint32_t configuredNodes = nod;
//   uint32_t total = knownDevices > configuredNodes ? knownDevices : configuredNodes;
//   if (total == 0)
//   {
//     total = DEFAULT_TARGET_GROUP_COUNT;
//   }
//   return total;
// }

// inline void refreshGroupConfiguration()
// {
//   groupConfig.totalNodes = getPlannedNodeCount();

//   uint8_t desiredSize = groupConfig.groupSize;
//   if (desiredSize == 0)
//   {
//     uint32_t targetCount = DEFAULT_TARGET_GROUP_COUNT;
//     if (targetCount == 0)
//     {
//       targetCount = 1;
//     }
//     desiredSize = (groupConfig.totalNodes + targetCount - 1) / targetCount;
//   }

//   if (groupConfig.totalNodes > 0 && desiredSize > groupConfig.totalNodes)
//   {
//     desiredSize = static_cast<uint8_t>(groupConfig.totalNodes);
//   }

//   if (desiredSize == 0)
//   {
//     desiredSize = 1;
//   }

//   groupConfig.groupSize = desiredSize;

//   uint32_t computedCount = (groupConfig.totalNodes + desiredSize - 1) / desiredSize;
//   if (computedCount == 0)
//   {
//     computedCount = 1;
//   }
//   if (computedCount > 255)
//   {
//     computedCount = 255;
//   }
//   groupConfig.groupCount = static_cast<uint8_t>(computedCount);

//   if (groupConfig.responseWindowMs == 0)
//   {
//     groupConfig.responseWindowMs = DEFAULT_GROUP_RESPONSE_WINDOW_MS;
//   }
//   if (groupConfig.responseWindowMs < MIN_GROUP_RESPONSE_WINDOW_MS)
//   {
//     groupConfig.responseWindowMs = MIN_GROUP_RESPONSE_WINDOW_MS;
//   }
// }

// inline uint8_t ensureDeviceGroup(int index)
// {
//   if (index < 0 || index >= Device.deviceCount)
//   {
//     return 1;
//   }

//   if (Device.groupId[index] == 0)
//   {
//     Device.groupId[index] = resolveGroupIdForIndex(index, 0);
//   }
//   return Device.groupId[index];
// }

// inline bool hasPendingResponsesInGroup(uint8_t groupId)
// {
//   for (int i = 0; i < Device.deviceCount; i++)
//   {
//     if (!Device.pendingResponse[i])
//     {
//       continue;
//     }

//     uint8_t deviceGroup = ensureDeviceGroup(i);
//     if (groupId == 0 || deviceGroup == groupId)
//     {
//       return true;
//     }
//   }
//   return false;
// }

// inline void appendGroupConfiguration(DynamicJsonDocument &dataDoc, uint8_t targetGroupId, bool includeAssignments)
// {
//   JsonObject groupCfg = dataDoc.createNestedObject("group_cfg");
//   groupCfg["group_count"] = groupConfig.groupCount;
//   groupCfg["group_size"] = groupConfig.groupSize;
//   groupCfg["total_nodes"] = groupConfig.totalNodes;
//   groupCfg["response_window_ms"] = groupConfig.responseWindowMs;
//   groupCfg["assignment"] = "by_index";
//   if (targetGroupId != 0)
//   {
//     groupCfg["target_group_id"] = targetGroupId;
//   }

//   // Gợi ý cho node biết các trường cần phản hồi để Hub có thể theo dõi nhóm chính xác.
//   JsonObject responseHint = groupCfg.createNestedObject("response_hint");
//   responseHint["require_group_id"] = true;
//   responseHint["fallback_policy"] = "by_assignment";
//   if (targetGroupId != 0)
//   {
//     responseHint["default_group_id"] = targetGroupId;
//   }

//   uint8_t size = groupConfig.groupSize == 0 ? 1 : groupConfig.groupSize;
//   uint32_t nodes = groupConfig.totalNodes;
//   if (nodes == 0)
//   {
//     nodes = size * groupConfig.groupCount;
//     if (nodes == 0)
//     {
//       nodes = size;
//     }
//   }

//   JsonArray groups = groupCfg.createNestedArray("groups");
//   for (uint8_t gid = 1; gid <= groupConfig.groupCount; gid++)
//   {
//     JsonObject groupObj = groups.createNestedObject();
//     groupObj["id"] = gid;
//     uint32_t startIndex = (static_cast<uint32_t>(gid - 1) * size) + 1;
//     uint32_t endIndex = gid * size;
//     if (endIndex > nodes)
//     {
//       endIndex = nodes;
//     }
//     if (startIndex > nodes)
//     {
//       startIndex = nodes;
//     }
//     groupObj["start_index"] = startIndex;
//     groupObj["end_index"] = endIndex;
//   }

//   if (includeAssignments)
//   {
//     JsonArray assignments = groupCfg.createNestedArray("assignments");
//     size_t totalAssignments = Device.deviceCount;
//     size_t includedAssignments = totalAssignments;
//     if (includedAssignments > MAX_GROUP_ASSIGNMENTS_IN_MESSAGE)
//     {
//       includedAssignments = MAX_GROUP_ASSIGNMENTS_IN_MESSAGE;
//       groupCfg["assignments_truncated"] = true;
//     }
//     groupCfg["assignments_total"] = totalAssignments;
//     groupCfg["assignments_included"] = includedAssignments;
//     for (size_t i = 0; i < includedAssignments; i++)
//     {
//       JsonObject entry = assignments.createNestedObject();
//       entry["lid"] = Device.LocalID[i];
//       entry["gid"] = ensureDeviceGroup(i);
//     }
//   }
// }

// #endif // ESPNOW_GROUP_H



#ifndef ESPNOW_GROUP_H
#define ESPNOW_GROUP_H

#include <Arduino.h>
#include <ArduinoJson.h>

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

/// @brief Số nhóm mặc định mà Hub luôn cố gắng duy trì.
constexpr uint8_t DEFAULT_TARGET_GROUP_COUNT = 5;
/// @brief Giới hạn cứng số thiết bị trong một nhóm.
constexpr uint8_t MAX_DEVICES_PER_GROUP = 20;
/// @brief Thời gian chờ phản hồi mặc định cho mỗi nhóm (ms).
constexpr uint32_t DEFAULT_GROUP_RESPONSE_WINDOW_MS = 1500;
/// @brief Ngưỡng tối thiểu để tránh cấu hình thời gian chờ quá thấp (ms).
constexpr uint32_t MIN_GROUP_RESPONSE_WINDOW_MS = 500;
/// @brief Số bản ghi gán nhóm tối đa được gửi kèm trong một gói JSON.
constexpr size_t MAX_GROUP_ASSIGNMENTS_IN_MESSAGE = 24;

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

/**
 * @brief Gửi kèm thông tin cấu hình nhóm vào JSON trả về cho thiết bị.
 *
 * Cấu trúc JSON giúp node biết mình thuộc nhóm nào, khoảng index của từng nhóm
 * và (nếu cần) danh sách gán cụ thể, phục vụ cho việc lên lịch phản hồi.
 */
inline void appendGroupConfiguration(DynamicJsonDocument &dataDoc, uint8_t targetGroupId, bool includeAssignments)
{
  JsonObject groupCfg = dataDoc.createNestedObject("group_cfg");
  groupCfg["group_count"] = groupConfig.groupCount;
  groupCfg["group_size"] = groupConfig.groupSize;
  groupCfg["total_nodes"] = groupConfig.totalNodes;
  groupCfg["response_window_ms"] = groupConfig.responseWindowMs;
  groupCfg["assignment"] = "by_index";
  if (targetGroupId != 0)
  {
    groupCfg["target_group_id"] = targetGroupId;
  }

  // Gợi ý cho node biết các trường cần phản hồi để Hub có thể theo dõi nhóm chính xác.
  JsonObject responseHint = groupCfg.createNestedObject("response_hint");
  responseHint["require_group_id"] = true;
  responseHint["fallback_policy"] = "by_assignment";
  if (targetGroupId != 0)
  {
    responseHint["default_group_id"] = targetGroupId;
  }

  uint8_t size = groupConfig.groupSize == 0 ? 1 : groupConfig.groupSize;
  uint32_t nodes = groupConfig.totalNodes;
  if (nodes == 0)
  {
    nodes = size * groupConfig.groupCount;
    if (nodes == 0)
    {
      nodes = size;
    }
  }
  uint32_t maxNodes = static_cast<uint32_t>(size) * groupConfig.groupCount;
  if (nodes > maxNodes && maxNodes > 0)
  {
    nodes = maxNodes;
  }

  JsonArray groups = groupCfg.createNestedArray("groups");
  for (uint8_t gid = 1; gid <= groupConfig.groupCount; gid++)
  {
    JsonObject groupObj = groups.createNestedObject();
    groupObj["id"] = gid;
    uint32_t startIndex = (static_cast<uint32_t>(gid - 1) * size) + 1;
    uint32_t endIndex = gid * size;
    if (endIndex > nodes)
    {
      endIndex = nodes;
    }
    if (startIndex > nodes)
    {
      startIndex = nodes;
    }
    groupObj["start_index"] = startIndex;
    groupObj["end_index"] = endIndex;
  }

  if (includeAssignments)
  {
    JsonArray assignments = groupCfg.createNestedArray("assignments");
    size_t totalAssignments = Device.deviceCount;
    size_t includedAssignments = totalAssignments;
    if (includedAssignments > MAX_GROUP_ASSIGNMENTS_IN_MESSAGE)
    {
      includedAssignments = MAX_GROUP_ASSIGNMENTS_IN_MESSAGE;
      groupCfg["assignments_truncated"] = true;
    }
    groupCfg["assignments_total"] = totalAssignments;
    groupCfg["assignments_included"] = includedAssignments;
    for (size_t i = 0; i < includedAssignments; i++)
    {
      JsonObject entry = assignments.createNestedObject();
      entry["lid"] = Device.LocalID[i];
      entry["gid"] = ensureDeviceGroup(i);
    }
  }
}

#endif // ESPNOW_GROUP_H