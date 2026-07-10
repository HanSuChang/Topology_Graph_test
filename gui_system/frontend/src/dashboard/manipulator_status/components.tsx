import { Card, Badge } from "@/dashboard/layout/Card";

export function GripperStateCard() {
  return (
    <Card title="Manipulator">
      <div className="flex gap-2 items-center mb-2">
        <Badge tone="muted">Idle</Badge>
        <Badge tone="muted">Gripper: Open</Badge>
      </div>
      <div className="text-sm">Pose: x=0.00  y=0.00  z=0.20</div>
    </Card>
  );
}

export function PickDropStatus() {
  return (
    <Card title="Pick / Drop">
      <div className="text-sm space-y-1">
        <div>Last Pick: —</div>
        <div>Last Drop: —</div>
      </div>
    </Card>
  );
}

export function RecoveryModePanel() {
  return (
    <Card title="Recovery (admin)">
      <div className="text-sm text-slate-400">
        Should 단계에서 활성화: Gripper Open / Close / Home Pose. 관리자 인증 후만 사용.
      </div>
    </Card>
  );
}
