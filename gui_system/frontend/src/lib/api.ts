// Go REST 표면에 대한 얇은 fetch 래퍼. 관리자 명령 경로가
// /api/v1/auth/login이 설정한 session_id 쿠키에 의존하므로 쿠키는 기본으로
// 함께 전송된다.

import type { SystemState } from "@/types/domain/systemState";
import type { RobotState } from "@/types/domain/robotState";
import type {
  HealthResponse,
  LoginResponse,
  CommandResult,
  MissionHistoryRow,
} from "@/types/api/response";
import type { ChangeGoalRequest, StartMissionRequest } from "@/types/api/request";
import { newRequestId } from "./idempotency";

const API = "/api/v1";

async function req<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(API + path, {
    credentials: "include",
    headers: {
      "Content-Type": "application/json",
      "X-Request-ID": newRequestId(),
      ...(init?.headers || {}),
    },
    ...init,
  });
  if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
  return res.json();
}

export const api = {
  health: () => req<HealthResponse>("/health"),
  stateCurrent: () => req<SystemState>("/state/current"),
  stateSystem: () => req<Omit<SystemState, "robots">>("/state/system"),
  stateRobots: () => req<{ robots: RobotState[] }>("/state/robots"),
  stateMap: () => req<{ nodes: any[]; edges: any[]; robots: any[] }>("/state/map"),
  missionHistory: () => req<MissionHistoryRow[]>("/missions"),

  login: (password_hash: string, username?: string) =>
    req<LoginResponse>("/auth/login", {
      method: "POST",
      body: JSON.stringify({ password_hash, username }),
    }),

  startMission: (body: StartMissionRequest) =>
    req<CommandResult>("/missions/start", { method: "POST", body: JSON.stringify(body) }),
  pause: () => req<CommandResult>("/missions/pause", { method: "POST" }),
  resume: () => req<CommandResult>("/missions/resume", { method: "POST" }),
  reset: () => req<CommandResult>("/missions/reset", { method: "POST" }),
  changeGoal: (body: ChangeGoalRequest) =>
    req<CommandResult>("/missions/goal", { method: "POST", body: JSON.stringify(body) }),
  emergencyStop: () => req<CommandResult>("/emergency/stop", { method: "POST" }),
  emergencyClear: () => req<CommandResult>("/emergency/clear", { method: "POST" }),
  // RViz 스타일 2D Pose Estimate. 월드 좌표(x/y/theta)는 브릿지 →
  // /initialpose 발행을 구동하고, 정규화 캔버스 좌표(nx/ny + dx_n/dy_n)는
  // 다중 클라이언트 시각 미러를 구동해 PC와 폰이 같은 이미지 콘텐츠에서
  // 마커를 보게 한다.
  poseEstimate: (body: {
    x: number;
    y: number;
    theta: number;
    nx?: number;
    ny?: number;
    dx_n?: number;
    dy_n?: number;
    robot_id?: string;
  }) =>
    req<CommandResult>("/pose_estimate", { method: "POST", body: JSON.stringify(body) }),
};
