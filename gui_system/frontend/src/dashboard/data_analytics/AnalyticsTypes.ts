// 대시보드 분석 표면용 로컬 DTO. internal/api/data_analytics에서 제공되는
// Go HistoryRow + 차트 집계를 반영한다. 정식 API 계약이 아니라
// 대시보드 전용 투영이라 src/types/api가 아닌 여기에 둔다.

export interface MissionRow {
  id: string;
  type: string;
  status: string;
  target_node: string;
  start_time?: string;
  end_time?: string;
  predicted_eta?: number;
  actual_eta?: number;
}

export interface EtaSample {
  id: string;
  predicted: number;
  actual: number;
}

export interface TimeSeriesPoint {
  ts: number;
  value: number;
}
