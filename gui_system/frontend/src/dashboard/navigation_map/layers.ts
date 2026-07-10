// MapRenderer.render()가 소비하는 드로잉 프리미티브. 각 `draw*` 헬퍼는
// 합성 맵의 한 레이어를 그리는 작은 (거의) 순수 함수다. 이전에는 한
// 목적의 별도 파일(topologyLayer / pathLayer / robotPoseLayer /
// costmapLayer / localizationState)에 있었지만, 각 30~120 LoC이고 항상
// 함께 호출되어 — 다섯 파일이 줄여준 것보다 더 많은 인지 부담을 더했기에
// 여기로 합쳤다.

import type { MapEdge, MapNode, MapPath, MapPoint, MapRobot } from "./mapTypes";
import { toScreen, headingToScreenAngle, type Viewport } from "./coordinateTransform";

// ─── 토폴로지 레이어 (노드 + 엣지) ──────────────────────────────────

// 노드 타입 → 색. 기존 GUI 타입(home/storage/delivery/charging)과 함께,
// Topology_Graph_test 통합용 타입(loading_zone/intersection/area_entry/
// stationing_slot/charger/standby 등)도 다룬다. 미정의 타입은 회색.
const NODE_COLOR: Record<string, string> = {
  delivery: "#10b981",
  charging: "#f59e0b",
  home: "#3b82f6",
  storage: "#94a3b8",
  // topology.yaml 통합 타입
  loading_zone: "#10b981",
  charger: "#f59e0b",
  charger_entry: "#fbbf24",
  stationing_slot: "#3b82f6",
  stationing_slot_precision: "#60a5fa",
  standby: "#8b5cf6",
  intersection: "#64748b",
  area_entry: "#cbd5e1",
};

const NODE_COLOR_BY_ID: Record<string, string> = {
  loading: "#10b981",
  intersection_1: "#ef4444",
  a_entry: "#ef4444",
  rc_a_stop: "#ef4444",
  a_leader_slot: "#ef4444",
  intersection_2: "#3b82f6",
  b_entry: "#3b82f6",
  rc_b_stop: "#3b82f6",
  b_leader_slot: "#3b82f6",
  charger_entry: "#fbbf24",
  charger_front: "#f59e0b",
};

// NodeLight는 노드 라이트 상태다: target=현재 주행 대상(빨강), arrived=현재
// 도달 노드(초록). 라이트가 없는 노드는 평소 색(꺼짐).
export type NodeLight = "target" | "arrived";

// drawTopology는 정적 노드 + 엣지 그래프를 렌더한다. 설계 §6-1의 색
// 구성에 맞춰, 운영자가 한눈에 delivery / charging / storage를 구분할 수
// 있도록 노드를 타입별로 색칠한다. lights가 주어지면 해당 노드에 빨강/초록
// 글로우 링(라이트)을 켠다.
export function drawTopology(
  ctx: CanvasRenderingContext2D,
  vp: Viewport,
  nodes: MapNode[],
  edges: MapEdge[],
  lights: Map<string, NodeLight> = new Map(),
) {
  // 노드 원이 위에 렌더되도록 엣지를 먼저 그린다.
  ctx.strokeStyle = "rgba(15,23,42,0.72)";
  ctx.lineWidth = 1.5;
  for (const e of edges) {
    const a = nodes.find((n) => n.id === e.from);
    const b = nodes.find((n) => n.id === e.to);
    if (!a || !b) continue;
    const pa = toScreen(vp, a);
    const pb = toScreen(vp, b);
    ctx.beginPath();
    ctx.moveTo(pa.x, pa.y);
    ctx.lineTo(pb.x, pb.y);
    ctx.stroke();
  }

  for (const n of nodes) {
    const p = toScreen(vp, n);
    // 라이트(불) — target=빨강, arrived=초록. 노드 뒤에 글로우 디스크를 깔아
    // 노드에 색 링+발광이 생긴다. 라이트 없는 노드는 평소 색만(꺼짐).
    const light = lights.get(n.id);
    if (light) {
      const color = light === "target" ? "#ef4444" : "#22c55e";
      ctx.save();
      ctx.shadowColor = color;
      ctx.shadowBlur = 14;
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 9, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }
    ctx.fillStyle = NODE_COLOR_BY_ID[n.id] ?? NODE_COLOR[n.type] ?? "#94a3b8";
    ctx.beginPath();
    ctx.arc(p.x, p.y, 6, 0, Math.PI * 2);
    ctx.fill();
    // 어떤 배경에서도 라벨이 읽히도록 흰색 halo.
    ctx.fillStyle = "rgba(255,255,255,0.85)";
    const text = n.name || n.id;
    const isRcStop = n.id === "rc_a_stop" || n.id === "rc_b_stop";
    ctx.font = `${isRcStop ? 10 : 11}px Pretendard, system-ui, monospace`;
    const w = ctx.measureText(text).width;
    const labelX = Math.max(2, Math.min(p.x + 6, ctx.canvas.width - w - 8));
    const labelY = Math.max(14, Math.min(p.y - 16, ctx.canvas.height - 16));
    ctx.fillRect(labelX, labelY, w + 6, 14);
    ctx.fillStyle = "#0f172a";
    ctx.fillText(text, labelX + 3, labelY + 10);
  }
}

// pickNode는 화면 클릭(16px 이내)에 가장 가까운 등록 노드를 반환하고,
// 빈 공간 클릭이면 null을 반환한다. 설계 §6-2에 따라 대시보드는 운영자가
// 임의 goal을 두는 것을 절대 허용하지 않는다.
export function pickNode(
  vp: Viewport,
  nodes: MapNode[],
  screenX: number,
  screenY: number,
): MapNode | null {
  let best: { n: MapNode; d: number } | null = null;
  for (const n of nodes) {
    const p = toScreen(vp, n);
    const d = Math.hypot(p.x - screenX, p.y - screenY);
    if (d < 16 && (!best || d < best.d)) best = { n, d };
  }
  return best?.n ?? null;
}

// ─── 경로 레이어 (global / local / trajectory) ───────────────────────

// drawPaths는 global / local / trajectory 폴리라인을 렌더한다.
//  - global (Nav2 plan):   옅은 하늘색, 점선 — "계획된 경로"
//  - local  (Nav2 local):  파랑, 실선        — 단기 horizon 보정
//  - trajectory (실제):    slate, 얇은 실선  — 로봇이 실제로 간 경로
// 범례 색은 mapTypes.PATH_COLORS에 있다.
export function drawPaths(
  ctx: CanvasRenderingContext2D,
  vp: Viewport,
  paths: Map<string, MapPath>,
) {
  for (const path of paths.values()) {
    if (path.points.length < 2) continue;
    ctx.save();
    ctx.strokeStyle = path.color;
    if (path.kind === "global") {
      ctx.lineWidth = 2.5;
      ctx.setLineDash([8, 6]);
      ctx.lineCap = "round";
    } else if (path.kind === "local") {
      // 동적 회피 호 — global(긴 점선 sky-blue)과 시각적으로 구분되도록 짧은
      // 점선 + 진한 파랑(#60a5fa, PATH_COLORS.local). 회피 종료 시 빈 points로
      // 호출되면 drawPaths의 length<2 가드가 자동 소거한다.
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 4]);
      ctx.lineCap = "round";
    } else {
      ctx.lineWidth = 1.5;
      ctx.setLineDash([]);
    }
    ctx.beginPath();
    const start = toScreen(vp, path.points[0]);
    ctx.moveTo(start.x, start.y);
    for (let i = 1; i < path.points.length; i++) {
      const p = toScreen(vp, path.points[i]);
      ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
    ctx.restore();
  }
}

// ─── 라이다 스캔 레이어 (점 클라우드) ────────────────────────────────

// drawScanPoints는 라이다 스캔 한 사이클을 작은 초록 점으로 렌더한다. 브릿지가
// /scan을 map 프레임 (x,y)로 변환해 보낸다(`topic_subscriber.py` `_on_scan`).
// 수백 개 점을 그리므로 arc/beginPath 대신 fillRect로 비용을 낮춘다. 알파를
// 약간 낮춰 점이 겹쳐도 벽/노드/로봇 가독성을 해치지 않는다. MapRenderer의
// 렌더 순서는 paths 다음, robots 이전이라 마커가 점 위에 얹힌다.
export function drawScanPoints(
  ctx: CanvasRenderingContext2D,
  vp: Viewport,
  scans: Map<string, MapPoint[]>,
) {
  ctx.save();
  ctx.fillStyle = "rgba(34,197,94,0.85)"; // Tailwind green-500
  // 3×3 px(중심 정렬). 2×2보다 가시성을 키워 벽 정합 시각 검증을 쉽게 한다.
  for (const pts of scans.values()) {
    for (const p of pts) {
      const sp = toScreen(vp, p);
      ctx.fillRect(sp.x - 1.5, sp.y - 1.5, 3, 3);
    }
  }
  ctx.restore();
}

// ─── 로봇 pose 레이어 (마커 + 군집 체인) ─────────────────────────────

// 맵 마커용 표시 라벨. wire 레벨 robot_id를 운영자가 요청한 한글 라벨로
// 치환하며, 알 수 없는 ID는 raw id로 폴백한다.
const ROBOT_LABEL: Record<string, string> = {
  tb3_leader: "터틀봇3",
  rc_car_follower: "RC카",
  follower_1: "팔로워1",
  follower_2: "팔로워2",
};

// shortestAngle은 두 각도의 최단 차이(-π..π)를 반환한다. heading 보간이
// +π/−π 경계에서 한 바퀴 빙 도는 대신 가장 가까운 방향으로 돌게 한다.
function shortestAngle(to: number, from: number): number {
  let d = (to - from) % (Math.PI * 2);
  if (d > Math.PI) d -= Math.PI * 2;
  if (d < -Math.PI) d += Math.PI * 2;
  return d;
}

// robotDisplay는 진행 중인 lerp를 반영해 마커가 "지금" 표시돼야 할 보간
// 위치·heading을 계산한다. drawRobots(매 프레임)와 updateRobotPose(새 pose
// 도착 시 from을 현재 위치로 전진)가 같은 수식을 공유해, pose 도착 간격이
// 불규칙해도(WiFi 지터·amcl 비주기) 마커가 되돌아가거나 멈추지 않는다.
// lerp 시간은 고정 200ms가 아니라 직전 도착 간격(lerpDur)을 쓴다.
export function robotDisplay(r: MapRobot, now: number): { x: number; y: number; theta: number } {
  if (r.target && r.lerpStart != null) {
    const dur = r.lerpDur && r.lerpDur > 0 ? r.lerpDur : 200;
    const t = Math.min(1, (now - r.lerpStart) / dur);
    const fromTheta = r.fromTheta ?? r.theta;
    const targetTheta = r.targetTheta ?? r.theta;
    return {
      x: r.x + (r.target.x - r.x) * t,
      y: r.y + (r.target.y - r.y) * t,
      theta: fromTheta + shortestAngle(targetTheta, fromTheta) * t,
    };
  }
  return { x: r.x, y: r.y, theta: r.fromTheta ?? r.theta };
}

// drawRobots는 애니메이션 프레임마다 호출된다. 위치·heading 모두 robotDisplay로
// 적응형 보간해, 업데이트가 저빈도·불규칙하게 와도 캔버스가 부드럽게 보인다.
export function drawRobots(
  ctx: CanvasRenderingContext2D,
  vp: Viewport,
  robots: Map<string, MapRobot>,
  now: number,
) {
  for (const r of robots.values()) {
    const d = robotDisplay(r, now);
    const displayX = d.x;
    const displayY = d.y;
    const p = toScreen(vp, { x: displayX, y: displayY });

    // 본체: heading 방향을 가리키는 화살표 / chevron. 별도 선을 따라가지
    // 않고도 운영자가 "로봇이 어디로 향하는지" 읽게 한다. 4점 모양: tip,
    // 왼쪽 날개, 뒤쪽 notch(살짝 안쪽), 오른쪽 날개.
    const tipDist = 13;
    const baseDist = 9;
    const baseSpread = 2.5;
    const notchDist = 3;
    // heading을 화면 각도로 투영(맵 CCW90 회전·stretch 반영). toScreen이 맵
    // 변환을 포함하므로 world theta를 그대로 cos/sin 하면 방향이 틀어진다.
    // 반환 각도는 화면 좌표계(+y 아래)라 sin 부호를 뒤집지 않고 그대로 쓴다.
    const a = headingToScreenAngle(vp, { x: displayX, y: displayY }, d.theta);
    const cosT = Math.cos(a);
    const sinT = Math.sin(a);
    const tipX = p.x + cosT * tipDist;
    const tipY = p.y + sinT * tipDist;
    const leftX = p.x + Math.cos(a + baseSpread) * baseDist;
    const leftY = p.y + Math.sin(a + baseSpread) * baseDist;
    const rightX = p.x + Math.cos(a - baseSpread) * baseDist;
    const rightY = p.y + Math.sin(a - baseSpread) * baseDist;
    const notchX = p.x - cosT * notchDist;
    const notchY = p.y - sinT * notchDist;
    ctx.beginPath();
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(leftX, leftY);
    ctx.lineTo(notchX, notchY);
    ctx.lineTo(rightX, rightY);
    ctx.closePath();
    ctx.fillStyle = r.color;
    ctx.fill();
    ctx.strokeStyle = "rgba(15,23,42,0.6)";
    ctx.lineWidth = 1.5;
    ctx.stroke();
    // 라벨 — heading과 무관하게 항상 마커 바로 아래에 중앙 정렬로 고정한다.
    // 마커가 회전해도 라벨은 움직이지 않으며, chevron 최대 반경(~13px) 아래로
    // 띄워 어떤 방향을 향해도 마커를 가리지 않는다. 밝은 맵 위 가독성을 위한
    // 흰색 halo + 어두운 텍스트.
    const label = ROBOT_LABEL[r.id] ?? r.id;
    ctx.font = "11px Pretendard, system-ui, monospace";
    const w = ctx.measureText(label).width;
    const boxW = w + 6;
    const boxH = 14;
    const boxTop = p.y + 15;
    ctx.fillStyle = "rgba(255,255,255,0.85)";
    ctx.fillRect(p.x - boxW / 2, boxTop, boxW, boxH);
    ctx.fillStyle = "#0f172a";
    ctx.fillText(label, p.x - w / 2, boxTop + 11);
  }
}

// drawSwarmRelation은 leader → RC카/follower 점선 체인을 오버레이해
// 오버레이해 운영자가 포메이션 순서를 한눈에 읽게 한다.
export function drawSwarmRelation(
  ctx: CanvasRenderingContext2D,
  vp: Viewport,
  robots: Map<string, MapRobot>,
) {
  const order = robots.has("rc_car_follower")
    ? ["tb3_leader", "rc_car_follower"]
    : ["tb3_leader", "follower_1", "follower_2"];
  const now = performance.now();
  const pts = order.map((id) => {
    const r = robots.get(id);
    if (!r) return null;
    const d = robotDisplay(r, now);
    return toScreen(vp, { x: d.x, y: d.y });
  });
  ctx.strokeStyle = "#facc15";
  ctx.setLineDash([4, 4]);
  if (pts[0] && pts[1]) {
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    ctx.lineTo(pts[1].x, pts[1].y);
    ctx.stroke();
  }
  if (pts[1] && pts[2]) {
    ctx.beginPath();
    ctx.moveTo(pts[1].x, pts[1].y);
    ctx.lineTo(pts[2].x, pts[2].y);
    ctx.stroke();
  }
  ctx.setLineDash([]);
}

// ─── Costmap 레이어 ──────────────────────────────────────────────────

// Costmap은 브릿지에서 최대 1Hz로 forward되는 거친 occupancy grid다
// (설계 §3-6). Must 데모는 costmap을 받지 않지만, MapRenderer의 draw
// 루프를 건드리지 않고 Should가 연결할 수 있도록 렌더링 API를 여기 둔다.
export interface Costmap {
  origin: { x: number; y: number };
  resolution: number;
  width: number;
  height: number;
  data: Uint8Array;
}

export function drawCostmap(
  ctx: CanvasRenderingContext2D,
  vp: Viewport,
  cm: Costmap | null,
) {
  if (!cm) return;
  const px = vp.scale * cm.resolution;
  for (let r = 0; r < cm.height; r += 4) {
    for (let c = 0; c < cm.width; c += 4) {
      const v = cm.data[r * cm.width + c];
      if (v === 0 || v === 255) continue;
      const alpha = Math.min(0.5, v / 200);
      const wx = cm.origin.x + c * cm.resolution;
      const wy = cm.origin.y + r * cm.resolution;
      const sx = vp.width / 2 + vp.originX + wx * vp.scale;
      const sy = vp.height / 2 + vp.originY - wy * vp.scale;
      ctx.fillStyle = `rgba(244, 63, 94, ${alpha})`;
      ctx.fillRect(sx, sy, px * 4, px * 4);
    }
  }
}

// ─── 위치추정 상태 ───────────────────────────────────────────────────

// LocalizationState는 설계 §6-4의 AMCL/TF 헬스 표시기를 반영한다.
// 대시보드는 색으로 구분된 단일 뱃지를 보여줘, 로봇이 벽에 부딪히기 전에
// 운영자가 "Lost"를 알아채게 한다.
export type LocalizationLevel = "normal" | "unstable" | "lost";

export interface LocalizationStatus {
  level: LocalizationLevel;
  reason: string;
}

const LOCALIZATION_NORMAL: LocalizationStatus = { level: "normal", reason: "" };

// classifyLocalization은 가장 최근 진단 신호를 읽어 구간화한다.
// covariance는 AMCL pose covariance trace, lastPoseAgeMs는 마지막 pose
// 메시지 이후 경과 시간, tfHealthy는 `/tf`가 최근 map → base_link
// transform을 발행했는지를 반영한다.
export function classifyLocalization(
  covariance: number,
  lastPoseAgeMs: number,
  tfHealthy: boolean,
): LocalizationStatus {
  if (!tfHealthy) return { level: "lost", reason: "tf transform missing" };
  if (lastPoseAgeMs > 5000) return { level: "lost", reason: "no pose in 5s" };
  if (covariance > 1.0) return { level: "unstable", reason: "covariance high" };
  if (lastPoseAgeMs > 1500) return { level: "unstable", reason: "pose lagging" };
  return LOCALIZATION_NORMAL;
}
