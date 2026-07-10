// Go 백엔드로 POST되는 요청 본문.

export interface LoginRequest {
  password_hash: string;
}

export interface StartMissionRequest {
  target_node: string;
  type?: string;
  item_id?: string;
}

export interface ChangeGoalRequest {
  target_node: string;
}
