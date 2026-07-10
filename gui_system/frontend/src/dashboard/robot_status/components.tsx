import { Card, Badge } from "@/dashboard/layout/Card";
import type { RobotState } from "@/types/domain/robotState";

export function TurtleBot3Card({ robot }: { robot: RobotState }) {
  return (
    <Card title={robot.robot_id}>
      <div className="flex items-center gap-2 mb-2">
        <Badge tone={robot.connection_state === "online" ? "ok" : "err"}>{robot.connection_state}</Badge>
        <Badge tone="muted">{robot.status}</Badge>
      </div>
      <PoseCard robot={robot} />
      <BatteryStatus robot={robot} />
      <div className="text-sm text-slate-400 mt-1">Node: {robot.current_node || "—"}</div>
    </Card>
  );
}

export function PoseCard({ robot }: { robot: RobotState }) {
  return (
    <div className="text-sm space-y-1">
      <div>x: {robot.pose.x.toFixed(2)} m</div>
      <div>y: {robot.pose.y.toFixed(2)} m</div>
      <div>θ: {((robot.pose.theta * 180) / Math.PI).toFixed(0)}°</div>
    </div>
  );
}

export function BatteryStatus({ robot }: { robot: RobotState }) {
  const pct = Math.max(0, Math.min(1, robot.battery)) * 100;
  const tone = pct > 30 ? "ok" : pct > 15 ? "warn" : "err";
  return (
    <div className="mt-2">
      <div className="flex justify-between text-xs">
        <span>Battery</span>
        <Badge tone={tone}>{pct.toFixed(0)}%</Badge>
      </div>
      <div className="w-full bg-slate-800 rounded h-2 mt-1">
        <div className="bg-emerald-500 h-2 rounded" style={{ width: `${pct}%` }} />
      </div>
    </div>
  );
}
