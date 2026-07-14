import { useEffect } from "react";
import { wsClient } from "@/lib/websocket";
import { logInfo, logWarn, emitFromRemote } from "@/lib/eventLog";
import { MissionControlCard } from "./cards/MissionControlCard";
import { PoseEstimateCard } from "./cards/PoseEstimateCard";
import { SummaryCard } from "./cards/SummaryCard";
import { RobotStatusCard } from "./cards/RobotStatusCard";
import { ManipulatorCard } from "./cards/ManipulatorCard";
import { LogsStrip } from "./cards/LogsStrip";
import { translateMission, translateAlert, isMotionEvent, translateMotion, robotLabel } from "./cards/shared";
import { MapPanel } from "./navigation_map/MapPanel";

// MainDashboardPage는 통합 단일 화면 뷰를 구성한다. 각 카드는 cards/
// 아래 자체 파일에 있으며, 이 컴포넌트는 반응형 레이아웃과 일회성
// WebSocket → 이벤트 로그 브리지만 소유한다.
export default function MainDashboardPage() {
  // gateway envelope을 통합 이벤트 로그로 한 번만 흘려보낸다. 구독을
  // 여기 두면 그 라이프사이클이 대시보드 mount에 묶인다.
  useEffect(() => {
    const ws = wsClient();
    const offMission = ws.on("mission_state", (e) => {
      const status = e.payload?.payload?.status ?? e.payload?.event_type;
      logInfo("mission", `미션 상태 변경: ${translateMission(status) ?? status}`);
    });
    const offAlert = ws.on("alert", (e) => {
      const et = e.payload?.event_type as string | undefined;
      if (et === "arm_vision_status" || et === "arm_servo_angles") return;
      // 주행 동작(motion_*)은 경보가 아니라 정보 로그로 — 로봇 이름 + 한글 동작.
      if (isMotionEvent(et)) {
        logInfo("주행", `${robotLabel(e.payload?.robot_id ?? e.robot_id)} ${translateMotion(et)}`);
        return;
      }
      logWarn("alert", translateAlert(et));
    });
    const offLog = ws.on("system_log", (e) => {
      const msg = e.payload?.message ?? e.payload?.event_type ?? "";
      logInfo("system", String(msg));
    });
    // 다중 클라이언트 로그 미러: 다른 브라우저/폰이 /api/v1/client_log로
    // 로그 항목을 보냈고 서버가 fan-out한다. emitFromRemote는 이 탭에서
    // 비롯된 레코드(client_id 일치)는 건너뛴다.
    const offClientLog = ws.on("client_log", (e) => {
      const p = e.payload as
        | { level?: string; source?: string; message?: string; client_id?: string; ts?: number }
        | undefined;
      if (!p) return;
      emitFromRemote({
        level: (p.level as any) ?? "INFO",
        source: p.source ?? "remote",
        message: p.message ?? "",
        client_id: p.client_id,
        ts: p.ts,
      });
    });
    return () => { offMission?.(); offAlert?.(); offLog?.(); offClientLog?.(); };
  }, []);

  // 레이아웃: 3-컬럼 상단 grid(레일 + 맵) 위에 전체 폭 로그 스트립. lg+
  // 에서는 flex 공간 공유로 뷰포트에 고정되고, lg 미만에서는 높이 고정이
  // 풀려 페이지가 세로 스크롤된다(설치된 PWA 폰은 스크롤로 모든 카드에
  // 도달).
  return (
    <div className="flex flex-col gap-3 lg:h-[calc(100vh-64px)] lg:overflow-hidden">
      <div className="grid grid-cols-12 gap-3 lg:flex-1 lg:min-h-0">
        <LeftRail />
        <MapPanel />
        <RightRail />
      </div>
      <LogsStrip />
    </div>
  );
}

// LeftRail — 미션 제어 + (모바일: 요약 / 데스크탑: 위치 추정) + 로봇 상태.
function LeftRail() {
  return (
    <div className="col-span-12 lg:col-span-3 flex flex-col gap-3 lg:min-h-0">
      <MissionControlCard />
      <SummaryCard className="lg:hidden" />
      <PoseEstimateCard className="hidden lg:block" />
      <RobotStatusCard />
    </div>
  );
}

// RightRail — (모바일: 위치 추정 / 데스크탑: 요약) + 매니퓰레이터 상태.
function RightRail() {
  return (
    <div className="col-span-12 lg:col-span-3 flex flex-col gap-3 lg:min-h-0">
      <PoseEstimateCard className="lg:hidden" />
      <SummaryCard className="hidden lg:block" />
      <ManipulatorCard />
    </div>
  );
}
