// Go 백엔드의 응답 형태.

export interface HealthResponse {
  server: string;
  bridge: string;
  db: string;
}

export interface LoginResponse {
  session_id: string;
}

export interface CommandResult {
  request_id: string;
  accepted: boolean;
  message?: string;
  data?: Record<string, unknown>;
}

export interface MissionHistoryRow {
  id: string;
  type: string;
  status: string;
  target_node: string;
  start_time?: string;
  end_time?: string;
  predicted_eta?: number;
  actual_eta?: number;
}
