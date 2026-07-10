import type { RobotState } from "./robotState";

export interface SystemState {
  mission_status: string;
  emergency_active: boolean;
  robots: RobotState[];
  current_mission_id?: string;
  current_goal_node?: string;
  eta_seconds?: number;
  // 현재 미션의 토폴로지 경로 노드 id 순서. 맵이 이 노드만 표시한다.
  route?: string[];
}
