// 대시보드 카드들이 공유하는 작은 프리미티브. 각자 몇 줄짜리이고 독립적
// 라이프사이클이 없는 순수 표현 헬퍼라 파일을 따로 두지 않고 한곳에 모았다.

import type { ButtonHTMLAttributes } from "react";

// 미션 제어 그리드가 쓰는 보조 버튼. 모바일에서 더 큰 탭 타겟(py-2),
// 데스크탑에서 py-1.5로 축소.
export function Btn({ children, ...rest }: ButtonHTMLAttributes<HTMLButtonElement>) {
  return (
    <button
      {...rest}
      className="py-2 lg:py-1.5 rounded-lg bg-white/70 hover:bg-white border border-line text-sm text-ink disabled:opacity-50"
    >
      {children}
    </button>
  );
}

// 요약 카드 내부에 쓰이는 Stat 타일.
export function Stat({ label, value }: { label: string; value?: string }) {
  return (
    <div className="bg-white/60 border border-line rounded-lg py-1.5">
      <div className="text-[11px] lg:text-[10px] uppercase tracking-wider text-muted">{label}</div>
      <div className="text-sm font-semibold text-ink truncate">{value ?? "—"}</div>
    </div>
  );
}

// 미션 상태 → 한글 라벨. 요약 카드와 mission_state 로그 브리지가 사용.
// 기존 범용 상태(pending/running/...)와 물류 미션 FSM 6단계(설계 §2)를 함께
// 다룬다. 백엔드 domain.MissionStatus 상수와 wire 문자열을 1:1로 맞춘다.
export function translateMission(s?: string): string | undefined {
  if (!s) return undefined;
  return ({
    pending: "대기",
    running: "진행 중",
    paused: "일시정지",
    completed: "완료",
    failed: "실패",
    aborted: "중단",
    // 물류 미션 FSM 6단계
    loading: "상차",
    formation_driving: "대열 주행",
    area_stationing: "구역 정차",
    sequential_unloading: "순차 하차",
    leader_return: "리더 복귀",
    mission_queue: "미션 대기",
  } as Record<string, string>)[s] ?? s;
}

// alert event_type → 한글 라벨. 브릿지/백엔드는 alert를 영어 코드로
// 보내므로(현재 브릿지가 보내는 건 obstacle_close), 로그창에 한글로 보이게
// 변환한다. 알 수 없는 코드는 raw 문자열로 폴백.
export function translateAlert(s?: string): string {
  if (!s) return "알림";
  return ({
    obstacle_close: "장애물 근접",
    low_battery: "배터리 부족",
    localization_lost: "위치추정 실패",
    localization_unstable: "위치추정 불안정",
    emergency_stop: "비상 정지",
    emergency_clear: "비상 해제",
  } as Record<string, string>)[s] ?? s;
}

// 주행 동작(motion_*) event_type → 한글. 브릿지가 odom twist로 분류해
// 동작이 바뀔 때만 보낸다(motion_forward/reverse/turn_left/turn_right/stopped).
const MOTION_LABEL: Record<string, string> = {
  motion_forward: "전진",
  motion_reverse: "후진",
  motion_turn_left: "좌회전",
  motion_turn_right: "우회전",
  motion_stopped: "정지",
};

// isMotionEvent는 alert 스트림에서 주행 동작 이벤트를 일반 경보와 구분한다.
export function isMotionEvent(eventType?: string): boolean {
  return !!eventType && eventType in MOTION_LABEL;
}

export function translateMotion(eventType?: string): string {
  return (eventType && MOTION_LABEL[eventType]) || "이동";
}

// 로봇 status → 한글 라벨. 기존 운영 상태(idle/moving/...)와 TurtleBot 리더가
// RC카에 발행하는 상태 토픽(NORMAL/SLOW/STOP/AVOIDING/STATIONING/UNLOADING/
// RETURN_ALLOWED/RETURNING)을 함께 다룬다. wire가 대/소문자 어느 쪽이든
// 오도록 소문자로 정규화해 조회한다. 백엔드 domain.RobotStatus와 1:1.
const STATUS_LABEL: Record<string, string> = {
  idle: "대기",
  moving: "이동",
  picking: "집기",
  error: "오류",
  stopped: "정지",
  // TurtleBot 상태 토픽
  normal: "정상 주행",
  slow: "감속",
  stop: "정지",
  avoiding: "회피 중",
  stationing: "구역 정차",
  unloading: "하차 중",
  return_allowed: "복귀 허가",
  returning: "복귀 중",
};

export function translateStatus(s?: string): string | undefined {
  if (!s) return undefined;
  return STATUS_LABEL[s.toLowerCase()] ?? s;
}

// status별 뱃지 색. 운영자가 한눈에 위험/주의/정상을 읽도록 톤을 나눈다:
// danger(정지·오류) / warn(감속·회피·집기) / info(정차·하차·복귀중) / ok(정상·이동) / neutral(대기).
const STATUS_TONE: Record<string, string> = {
  idle: "bg-slate-100/80 text-slate-600 border-slate-200",
  moving: "bg-emerald-100/80 text-emerald-700 border-emerald-200",
  normal: "bg-emerald-100/80 text-emerald-700 border-emerald-200",
  return_allowed: "bg-emerald-100/80 text-emerald-700 border-emerald-200",
  picking: "bg-amber-100/80 text-amber-700 border-amber-200",
  slow: "bg-amber-100/80 text-amber-700 border-amber-200",
  avoiding: "bg-amber-100/80 text-amber-700 border-amber-200",
  stationing: "bg-sky-100/80 text-sky-700 border-sky-200",
  unloading: "bg-sky-100/80 text-sky-700 border-sky-200",
  returning: "bg-sky-100/80 text-sky-700 border-sky-200",
  error: "bg-rose-100/80 text-rose-700 border-rose-200",
  stop: "bg-rose-100/80 text-rose-700 border-rose-200",
  stopped: "bg-rose-100/80 text-rose-700 border-rose-200",
};

export function statusBadgeClass(s?: string): string {
  const base = "shrink-0 px-1 py-0 rounded text-[12px] lg:text-[11px] font-medium border";
  const tone = (s && STATUS_TONE[s.toLowerCase()]) || "bg-slate-100/80 text-slate-600 border-slate-200";
  return `${base} ${tone}`;
}

// 미션 제어 목적지 노드 id → 한글 라벨. 토폴로지 노드명은 영어("A Leader
// Slot")라, 운영자 친화적 라벨로 치환한다. 미정의 id는 노드 name/​id로 폴백.
export const NODE_LABEL: Record<string, string> = {
  loading: "상차 구역",
  a_leader_slot: "A 구역",
  b_leader_slot: "B 구역",
  charger_front: "충전소",
  charger_entry: "충전 진입",
  standby_1: "대기 구역",
};

// 로그 메시지용 로봇 한글 이름. 맵 마커 라벨(navigation_map)과 별개로 두어
// 카드/로그 모듈이 캔버스 모듈을 끌어오지 않게 한다.
export function robotLabel(id?: string): string {
  if (!id) return "로봇";
  return ({
    tb3_leader: "터틀봇3",
    follower_1: "팔로워1",
    follower_2: "팔로워2",
    rc_car_follower: "팔로워",
  } as Record<string, string>)[id] ?? id;
}

// 배터리 잔량(0..1) → Tailwind 텍스트 색 클래스. 운영자가 한눈에 위험
// 수준을 읽도록 구간별로 색을 달리한다: 충분(녹) / 주의(황) / 부족(적).
export function batteryColor(battery: number): string {
  const pct = battery * 100;
  if (pct <= 20) return "text-rose-600";
  if (pct <= 50) return "text-amber-500";
  return "text-emerald-600";
}

// 로봇별 마커 색상. navigation_map ROBOT_COLORS를 반영하되, 상태 칩이
// 캔버스 모듈을 끌어오지 않도록 독립적으로 둔다. 키는 wire robot_id.
export const ROBOT_COLOR: Record<string, string> = {
  tb3_leader: "#06b6d4",
  follower_1: "#8b5cf6",
  follower_2: "#ec4899",
  rc_car_follower: "#8b5cf6",
};

// 정렬 키: leader는 0, follower_N은 N, 그 외는 뒤로.
export function robotRank(id: string): number {
  if (/leader/i.test(id)) return 0;
  const m = id.match(/follower[_-]?(\d+)/i);
  return m ? Number(m[1]) : 999;
}

// 팔로워만 추종 뱃지를 단다; 리더 칩은 생략.
export function isFollower(id: string): boolean {
  return /follower/i.test(id);
}
