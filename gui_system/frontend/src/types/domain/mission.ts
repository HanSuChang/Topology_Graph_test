export type MissionType = "delivery" | "patrol" | "test";
// 범용 상태 + 물류 미션 FSM 6단계(설계 §2). 백엔드 domain.MissionStatus와 1:1.
export type MissionStatus =
  | "pending"
  | "running"
  | "paused"
  | "completed"
  | "failed"
  | "aborted"
  | "loading"
  | "formation_driving"
  | "area_stationing"
  | "sequential_unloading"
  | "leader_return"
  | "mission_queue";

export interface Mission {
  id: string;
  type: MissionType;
  item_id?: string;
  waypoints: string[];
  assigned_robots: string[];
  start_node: string;
  target_node: string;
  priority?: number;
  status: MissionStatus;
  current_task_index: number;
  start_time?: string;
  end_time?: string;
}
