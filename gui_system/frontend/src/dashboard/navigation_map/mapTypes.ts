// 네비게이션 맵 모듈 공유 타입. 월드 좌표는 미터(ROS 컨벤션), 화면
// 좌표는 픽셀이다.

export type MapPoint = { x: number; y: number };

export interface MapNode {
  id: string;
  name: string;
  x: number;
  y: number;
  type: string;
}

export interface MapEdge {
  from: string;
  to: string;
}

export interface MapRobot {
  id: string;
  // x, y는 현재 lerp의 시작(from) 위치. theta는 마지막으로 수신한 heading.
  x: number;
  y: number;
  theta: number;
  color: string;
  // target/targetTheta는 마지막으로 수신한 pose(보간 목표). fromTheta는 lerp
  // 시작 시점의 heading. lerpStart는 시작 시각, lerpDur은 적응형 보간 시간(ms,
  // 직전 업데이트 간격으로 결정), lastUpdate는 도착 간격 측정용 마지막 갱신 시각.
  target?: MapPoint;
  targetTheta?: number;
  fromTheta?: number;
  lerpStart?: number;
  lerpDur?: number;
  lastUpdate?: number;
}

export type PathKind = "global" | "local" | "trajectory";

export interface MapPath {
  kind: PathKind;
  points: MapPoint[];
  color: string;
}

export const ROBOT_COLORS: Record<string, string> = {
  tb3_leader: "#22d3ee",
  rc_car_follower: "#8b5cf6",
  follower_1: "#a78bfa",
  follower_2: "#f472b6",
};

export const PATH_COLORS: Record<PathKind, string> = {
  // 옅은 하늘색 점선 — /<ns>/plan에서 오는 Nav2 global plan. layers.ts에서
  // 점선 스타일로 그려, 운영자가 로컬 경로 / 실제 궤적과 혼동하지 않고
  // 로봇이 무엇을 하려는지 볼 수 있게 한다.
  global: "#7dd3fc",
  local: "#60a5fa",
  trajectory: "#94a3b8",
};
