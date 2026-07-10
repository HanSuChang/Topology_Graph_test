import { Card, Badge } from "@/dashboard/layout/Card";
import type { SystemState } from "@/types/domain/systemState";

type MissionLite = Omit<SystemState, "robots"> | null;

export function MissionStateCard({ mission }: { mission: MissionLite }) {
  return (
    <Card title="Mission">
      <div className="text-2xl">{mission?.mission_status ?? "—"}</div>
      <div className="text-sm text-slate-400 mt-1">
        Goal: {mission?.current_goal_node ?? "—"}
      </div>
    </Card>
  );
}

export function SystemStatusCard({ mission }: { mission: MissionLite }) {
  return (
    <Card title="System">
      {mission?.emergency_active ? (
        <Badge tone="err">EMERGENCY ACTIVE</Badge>
      ) : (
        <Badge tone="ok">nominal</Badge>
      )}
    </Card>
  );
}

export function EtaCard({ mission }: { mission: MissionLite }) {
  return (
    <Card title="ETA">
      <div className="text-2xl">
        {mission?.eta_seconds ? `${Math.round(mission.eta_seconds)}s` : "—"}
      </div>
    </Card>
  );
}

export function ProgressBar({ value }: { value: number }) {
  const pct = Math.max(0, Math.min(1, value)) * 100;
  return (
    <div className="w-full bg-slate-800 rounded h-2">
      <div className="bg-accent h-2 rounded" style={{ width: `${pct}%` }} />
    </div>
  );
}
