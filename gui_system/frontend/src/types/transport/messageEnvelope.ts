// MessageEnvelope은 모든 gateway WebSocket 프레임이 쓰는 JSON 형태다.
// Go `gateway.Envelope` struct와 정확히 일치한다.
export interface MessageEnvelope {
  type: string;
  robot_id?: string;
  timestamp: number;
  seq: number;
  request_id?: string;
  version: number;
  payload?: any;
}
