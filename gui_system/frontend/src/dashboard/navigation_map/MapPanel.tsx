import { useRef, useState } from "react";
import { Card } from "@/dashboard/layout/Card";
import { api } from "@/lib/api";
import { wsClient } from "@/lib/websocket";
import { logInfo, logErr } from "@/lib/eventLog";
import { currentMissionTarget, subscribeMissionTarget } from "@/lib/missionSelection";
import * as poseEstimate from "@/lib/poseEstimate";
import { useCommandGuard } from "@/dashboard/mission_control/hooks";
import { MissionCommandDialog } from "@/dashboard/mission_control/MissionCommandDialog";
import { MapCanvas } from "./MapCanvas";
import type { MapRenderer } from "./MapRenderer";

const TARGET_ROUTES: Record<string, string[]> = {
  loading: ["loading"],
  a_leader_slot: ["loading", "intersection_1", "a_entry", "rc_a_stop", "a_leader_slot"],
  b_leader_slot: ["loading", "intersection_2", "b_entry", "rc_b_stop", "b_leader_slot"],
  charger_front: ["loading", "charger_entry", "charger_front"],
};

function routeForTarget(target: string): string[] {
  return TARGET_ROUTES[target] ?? [];
}

// MapPanel은 네비게이션 맵을 소유한다: SLAM 배경, 토폴로지, 로봇 마커,
// 경로 계획, RViz 스타일 위치추정 인터랙션. MapRenderer(명령형 캔버스
// 클래스)를 WebSocket 스트림과 위치추정 토글에 연결한다.
export function MapPanel() {
  const rendererRef = useRef<MapRenderer | null>(null);
  const [selected, setSelected] = useState<string | null>(null);
  const { busy, run, needsAuth, dismissAuth, setMsg } = useCommandGuard();
  const pendingPoseRef = useRef<{
    x: number;
    y: number;
    theta: number;
    startedAt: number;
    logged: boolean;
    timer: number;
  } | null>(null);

  const onReady = (renderer: MapRenderer) => {
    rendererRef.current = renderer;
    renderer.loadSlamMap();
    api.stateMap().then((m) => {
      renderer.setTopology(
        (m.nodes ?? []).map((n: any) => ({ id: n.node_id ?? n.id, name: n.name, x: n.x, y: n.y, type: n.type })),
        (m.edges ?? []).map((e: any) => ({ from: e.from_node, to: e.to_node })),
      );
      renderer.setRoute(routeForTarget(currentMissionTarget()));
    }).catch(() => {});
    subscribeMissionTarget((target) => renderer.setRoute(routeForTarget(target)));
    // PoseEstimateCard의 위치추정 모드 토글을 렌더러 입력 핸들러로 연결한다.
    poseEstimate.subscribe((active) => renderer.setPoseEstimateMode(active));
    renderer.onPoseEstimate(async (x, y, theta, nx, ny, dxN, dyN) => {
      // 브릿지 왕복이 느려도 즉각적인 피드백을 위해 mouse-up 시 즉시
      // 비활성화한다. 다음 추정에는 버튼을 다시 눌러야 한다.
      poseEstimate.setActive(false);
      renderer.allowPoseJumpFor(5000);
      if (pendingPoseRef.current) {
        window.clearTimeout(pendingPoseRef.current.timer);
      }
      const pending = {
        x,
        y,
        theta,
        startedAt: Date.now(),
        logged: false,
        timer: window.setTimeout(() => {
          const p = pendingPoseRef.current;
          if (!p || p.logged || p.x !== x || p.y !== y) return;
          pendingPoseRef.current = null;
          logErr("pose", "위치 추정 반영 확인 실패: AMCL/TF pose 갱신이 감지되지 않음");
        }, 8000),
      };
      pendingPoseRef.current = pending;
      logInfo("pose", `위치 추정 전송 (${x.toFixed(2)}, ${y.toFixed(2)}, ${(theta * 180 / Math.PI).toFixed(0)}°)`);
      try {
        // 정규화 캔버스 좌표를 월드 좌표와 함께 보내, 다중 클라이언트
        // 시각 미러가 같은 이미지 콘텐츠에 안착하게 한다.
        const res = await api.poseEstimate({ x, y, theta, nx, ny, dx_n: dxN, dy_n: dyN }) as any;
        const message = String(res?.message ?? "");
        if (res?.bridge_offline || message.includes("stub initialpose")) {
          if (pendingPoseRef.current === pending) {
            window.clearTimeout(pending.timer);
            pendingPoseRef.current = null;
          }
          logErr("pose", "위치 추정 실패: ROS2 브릿지가 /initialpose에 연결되지 않음");
          return;
        }
        logInfo("pose", "위치 추정 값 전송됨: AMCL/TF 반영 대기");
      } catch (err: any) {
        if (pendingPoseRef.current === pending) {
          window.clearTimeout(pending.timer);
          pendingPoseRef.current = null;
        }
        logErr("pose", `위치 추정 실패: ${err?.message ?? "오류"}`);
      }
    });
    const ws = wsClient();
    ws.on("robot_pose", (e) => {
      const rs = e.payload;
      if (!rs?.robot_id || !rs?.pose) return;
      renderer.updateRobotPose(rs.robot_id, rs.pose.x, rs.pose.y, rs.pose.theta ?? 0);
      const pending = pendingPoseRef.current;
      if (!pending || pending.logged) return;
      const age = Date.now() - pending.startedAt;
      if (age < 150 || age > 8000) return;
      const dx = Number(rs.pose.x) - pending.x;
      const dy = Number(rs.pose.y) - pending.y;
      if (!Number.isFinite(dx) || !Number.isFinite(dy)) return;
      const distance = Math.hypot(dx, dy);
      if (distance <= 0.75) {
        pending.logged = true;
        window.clearTimeout(pending.timer);
        pendingPoseRef.current = null;
        logInfo("pose", `위치 추정 반영 확인됨 (${rs.robot_id}, 오차 ${distance.toFixed(2)}m)`);
      }
    });
    ws.on("path_data", (e) => {
      const p = e.payload;
      if (!p?.robot_id) return;
      renderer.updatePath(p.robot_id, p.kind ?? "global", p.points ?? []);
    });
    // 라이다 스캔 점 클라우드(map 프레임). 브릿지가 /scan을 TF로 변환해 보낸다.
    ws.on("scan_points", (e) => {
      const p = e.payload;
      if (!p?.robot_id || !Array.isArray(p.points)) return;
      renderer.updateScan(p.robot_id, p.points);
    });
    // 동적 장애물 우회 호(주행로직 DWA가 선택). path_data와 같은 경로 레이어에
    // kind="local"로 적재 → drawPaths가 파란 점선으로 표시. 빈 points = 회피 종료.
    ws.on("local_path", (e) => {
      const p = e.payload;
      if (!p?.robot_id) return;
      renderer.updatePath(p.robot_id, "local", p.points ?? []);
    });
    // 주행로직이 산출한 전역 계획 경로. backend mission.go의 로컬 Dijkstra가
    // path_data(kind=global)로 보내는 것과 같은 키(`robotId:global`)에 latest-
    // wins로 적재 → C++ publisher 활성화 시 자동 전환. 미션 종료=빈 points.
    ws.on("planned_path", (e) => {
      const p = e.payload;
      if (!p?.robot_id) return;
      renderer.updatePath(p.robot_id, "global", p.points ?? []);
    });
    // 다른 클라이언트가 확정한 위치추정을 미러링한다.
    ws.on("pose_estimate", (e) => {
      const p = e.payload as { x?: number; y?: number; theta?: number } | undefined;
      if (!p || typeof p.x !== "number" || typeof p.y !== "number") return;
      renderer.showRemotePose(p.x, p.y, p.theta ?? 0);
    });
    // 미션 경로 적재 → 맵이 경로 노드만 표시하고 노드 라이트를 켠다.
    // 초기 1회 /state/current로 현재 경로를 받고, 이후 mission_state로 갱신.
    const applyRoute = (route?: unknown) => {
      if (Array.isArray(route) && route.length > 0) {
        renderer.setRoute(route as string[]);
        return;
      }
      renderer.setRoute(routeForTarget(currentMissionTarget()));
    };
    api.stateCurrent().then((s) => applyRoute((s as any).route)).catch(() => {});
    ws.on("mission_state", (e) => {
      const p = (e.payload as any)?.payload ?? {};
      applyRoute(p.route);
    });
  };

  const sendGoal = () => {
    if (!selected) return;
    run(async () => {
      logInfo("ui", `맵에서 목적지 선택: ${selected}`);
      try {
        await api.startMission({ target_node: selected });
        logInfo("command", `주행 시작 수락됨 → ${selected}`);
      } catch (e: any) {
        logErr("command", `주행 시작 실패: ${e?.message ?? "오류"}`);
        throw e;
      }
    });
  };

  return (
    <Card
      title="네비게이션 맵"
      className="col-span-12 lg:col-span-6 flex flex-col lg:min-h-0"
      action={
        selected ? (
          <div className="flex items-center gap-2 text-xs">
            <code className="px-2 py-0.5 rounded bg-white/70 border border-line text-ink">{selected}</code>
            <button
              disabled={busy}
              onClick={sendGoal}
              className="px-2.5 py-1 rounded-md bg-accent text-white disabled:opacity-40"
            >
              해당 노드로 미션 시작
            </button>
          </div>
        ) : (
          <span className="text-[11px] text-muted">노드를 클릭해 목적지를 선택하세요</span>
        )
      }
    >
      <div className="flex-1 lg:min-h-0 max-lg:h-[60vh] max-lg:min-h-[360px]">
        <MapCanvas onReady={onReady} onNodeClick={(n) => setSelected(n.id)} />
      </div>
      <div className="text-[11px] text-muted mt-1.5">
        휠로 줌 · 드래그로 이동 · 등록된 노드만 목적지로 선택 가능 (자유 좌표 입력 불가)
      </div>
      <MissionCommandDialog
        open={needsAuth}
        onClose={dismissAuth}
        onAuthed={() => { dismissAuth(); setMsg("로그인 완료"); logInfo("auth", "관리자 로그인 완료"); }}
      />
    </Card>
  );
}
