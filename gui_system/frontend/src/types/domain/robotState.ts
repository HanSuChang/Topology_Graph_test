export type ConnectionState = "online" | "offline" | "lost";
// 기존 운영 상태 + TurtleBot 리더가 RC카에 발행하는 상태 토픽(대문자)을 함께
// 포함한다. 백엔드 domain.RobotStatus 상수와 wire 문자열을 1:1로 맞춘다.
export type RobotStatus =
  | "idle"
  | "moving"
  | "picking"
  | "error"
  | "stopped"
  | "NORMAL"
  | "SLOW"
  | "STOP"
  | "AVOIDING"
  | "STATIONING"
  | "UNLOADING"
  | "RETURN_ALLOWED"
  | "RETURNING";

export interface Pose {
  x: number;
  y: number;
  theta: number;
}

export interface RobotState {
  robot_id: string;
  pose: Pose;
  battery: number;
  status: RobotStatus;
  current_node: string;
  connection_state: ConnectionState;
  last_update: string;
}
