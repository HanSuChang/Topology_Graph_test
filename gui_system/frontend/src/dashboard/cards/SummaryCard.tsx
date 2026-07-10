import { Card } from "@/dashboard/layout/Card";
import { useMission } from "@/hooks/useMission";
import { Stat, translateMission, NODE_LABEL } from "./shared";

// SummaryCard는 미션 상태 스냅샷(미션 상태, ETA, 목적지)이다. 브레이크
// 포인트로 토글되는 가시성으로 양쪽 레일에 렌더된다 — 데스크탑은 우측,
// 모바일은 좌측.
export function SummaryCard({ className = "" }: { className?: string }) {
  const mission = useMission();
  return (
    <Card title="요약" className={`shrink-0 ${className}`}>
      <div className="grid grid-cols-3 gap-2 text-center">
        <Stat label="미션" value={translateMission(mission?.mission_status)} />
        <Stat label="예상 시간" value={mission?.eta_seconds ? `${Math.round(mission.eta_seconds)}초` : "—"} />
        <Stat label="목적지" value={mission?.current_goal_node ? (NODE_LABEL[mission.current_goal_node] ?? mission.current_goal_node) : undefined} />
      </div>
    </Card>
  );
}
