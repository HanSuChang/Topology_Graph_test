import { useMission } from "@/hooks/useMission";
import { useRobotState } from "@/hooks/useRobotState";
import { Card, Badge } from "@/dashboard/layout/Card";
import { MissionStateCard, EtaCard, SystemStatusCard, ProgressBar } from "./components";

export default function OverviewPage() {
  const mission = useMission();
  const robots = useRobotState();
  return (
    <div className="space-y-4">
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <MissionStateCard mission={mission} />
        <SystemStatusCard mission={mission} />
        <EtaCard mission={mission} />
      </div>
      <Card title="Robots">
        <ul className="space-y-1">
          {robots.length === 0 && <li className="text-slate-400">no robots</li>}
          {robots.map((r) => (
            <li key={r.robot_id} className="flex justify-between text-sm">
              <span>{r.robot_id}</span>
              <Badge tone={r.connection_state === "online" ? "ok" : "err"}>{r.connection_state}</Badge>
            </li>
          ))}
        </ul>
      </Card>
      <Card title="Mission Progress">
        <ProgressBar value={mission?.eta_seconds ? 0.42 : 0} />
      </Card>
    </div>
  );
}
