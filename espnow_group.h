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

constexpr uint8_t DEFAULT_TARGET_GROUP_COUNT = 4;
constexpr uint32_t DEFAULT_GROUP_RESPONSE_WINDOW_MS = 1500;
constexpr uint32_t MIN_GROUP_RESPONSE_WINDOW_MS = 500;
constexpr size_t MAX_GROUP_ASSIGNMENTS_IN_MESSAGE = 24;

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

  return (index / size) + 1;
}

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

inline void refreshGroupConfiguration()
{
  groupConfig.totalNodes = getPlannedNodeCount();

  uint8_t desiredSize = groupConfig.groupSize;
  if (desiredSize == 0)
  {
    uint32_t targetCount = DEFAULT_TARGET_GROUP_COUNT;
    if (targetCount == 0)
    {
      targetCount = 1;
    }
    desiredSize = (groupConfig.totalNodes + targetCount - 1) / targetCount;
  }

  if (groupConfig.totalNodes > 0 && desiredSize > groupConfig.totalNodes)
  {
    desiredSize = static_cast<uint8_t>(groupConfig.totalNodes);
  }

  if (desiredSize == 0)
  {
    desiredSize = 1;
  }

  groupConfig.groupSize = desiredSize;

  uint32_t computedCount = (groupConfig.totalNodes + desiredSize - 1) / desiredSize;
  if (computedCount == 0)
  {
    computedCount = 1;
  }
  if (computedCount > 255)
  {
    computedCount = 255;
  }
  groupConfig.groupCount = static_cast<uint8_t>(computedCount);

  if (groupConfig.responseWindowMs == 0)
  {
    groupConfig.responseWindowMs = DEFAULT_GROUP_RESPONSE_WINDOW_MS;
  }
  if (groupConfig.responseWindowMs < MIN_GROUP_RESPONSE_WINDOW_MS)
  {
    groupConfig.responseWindowMs = MIN_GROUP_RESPONSE_WINDOW_MS;
  }
}

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