// WS 메시지 type 판별자. backend/internal/gateway/envelope.go 상수와
// 동기화 유지해, 소비자가 switch dispatcher에서 string-literal 타입
// 좁히기에 의존할 수 있게 한다.
export const WS_TYPES = {
  ROBOT_POSE: "robot_pose",
  ROBOT_STATE: "robot_state",
  MISSION_STATE: "mission_state",
  PATH_DATA: "path_data",
  SCAN_POINTS: "scan_points",
  LOCAL_PATH: "local_path",
  PLANNED_PATH: "planned_path",
  SYSTEM_LOG: "system_log",
  ALERT: "alert",
  ETA_UPDATE: "eta_update",
  COMMAND_RESULT: "command_result",
  TELEMETRY: "telemetry",
} as const;

export type WsType = (typeof WS_TYPES)[keyof typeof WS_TYPES];
