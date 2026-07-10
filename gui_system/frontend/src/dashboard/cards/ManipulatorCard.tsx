import { Card, Badge } from "@/dashboard/layout/Card";

// ManipulatorCard는 암의 현재 자세 + 그리퍼 상태를 보여준다. 복구
// 제어(gripper open/close/home)는 관리자 인증으로 게이트되며 Should
// 단계에서 들어온다.
export function ManipulatorCard() {
  return (
    <Card title="매니퓰레이터 상태" className="shrink-0">
      <div className="flex gap-1.5 mb-2">
        <Badge tone="muted">대기</Badge>
        <Badge tone="muted">그리퍼: 열림</Badge>
      </div>
      <div className="text-xs space-y-0.5 text-muted">
        <div>현재 자세: x=0.00  y=0.00  z=0.20</div>
        <div>최근 Pick: —</div>
        <div>최근 Drop: —</div>
      </div>
      <div className="text-[11px] text-muted/70 mt-2">
        복구 모드(Gripper Open/Close/Home)는 관리자 인증 후 활성화됩니다.
      </div>
    </Card>
  );
}

// ManipulatorCameraCard는 비전급 프레임 슬롯을 예약한다. 설계 §5-4에 따라
// 프레임은 Pick 중에만 의미가 있으며, 카드가 남은 레일 높이를 채워
// 오버레이 아래에 빈 공간이 없게 한다.
export function ManipulatorCameraCard({ className = "flex-1 min-h-0" }: { className?: string }) {
  return (
    <Card title="매니퓰레이터 카메라" className={`${className} flex flex-col`}>
      <div className="relative w-full flex-1 min-h-0 rounded-lg bg-white/60 border border-line overflow-hidden">
        <span className="absolute inset-0 flex items-center justify-center text-[11px] text-muted pointer-events-none">
          Pick 시점 ±5초 또는 실패 스냅샷이 표시됩니다
        </span>
        <img
          src="/camera/manipulator/stream"
          alt="매니퓰레이터 카메라"
          className="relative w-full h-full object-cover"
          onError={(e) => { (e.target as HTMLImageElement).style.display = "none"; }}
        />
      </div>
    </Card>
  );
}
