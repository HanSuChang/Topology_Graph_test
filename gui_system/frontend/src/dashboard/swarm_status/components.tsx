import { Card, Badge } from "@/dashboard/layout/Card";
import type { RobotState } from "@/types/domain/robotState";

function dist(a?: RobotState, b?: RobotState) {
  if (!a || !b) return null;
  return Math.hypot(a.pose.x - b.pose.x, a.pose.y - b.pose.y);
}

export function FormationErrorCard({
  leader,
  f1,
  f2,
}: {
  leader?: RobotState;
  f1?: RobotState;
  f2?: RobotState;
}) {
  const d1 = dist(leader, f1);
  const d2 = dist(f1, f2);
  return (
    <Card title="Formation">
      <div className="text-lg mb-2">Leader → F1 → F2</div>
      <div className="text-sm">Leader → F1: {d1 != null ? `${d1.toFixed(2)} m` : "—"}</div>
      <div className="text-sm">F1 → F2: {d2 != null ? `${d2.toFixed(2)} m` : "—"}</div>
    </Card>
  );
}

export function FollowerStatusCard({ f1, f2 }: { f1?: RobotState; f2?: RobotState }) {
  return (
    <Card title="Followers">
      <div className="flex flex-col gap-2">
        <Row label="Follower 1" rs={f1} />
        <Row label="Follower 2" rs={f2} />
      </div>
    </Card>
  );
}

function Row({ label, rs }: { label: string; rs?: RobotState }) {
  return (
    <div className="flex justify-between">
      <span>{label}</span>
      {rs ? (
        <Badge tone={rs.connection_state === "online" ? "ok" : "err"}>{rs.connection_state}</Badge>
      ) : (
        <Badge tone="muted">—</Badge>
      )}
    </div>
  );
}

export function SwarmRelationView() {
  return (
    <Card title="Load">
      <div>적재 여부: <Badge tone="muted">Unknown</Badge></div>
      <div className="text-xs text-slate-500 mt-2">
        Manipulator 완료 이벤트 또는 운영자 확인 기반 (설계 §5-2).
      </div>
    </Card>
  );
}
