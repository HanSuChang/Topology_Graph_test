import { GripperStateCard, PickDropStatus, RecoveryModePanel } from "./components";

// 설계 §5-3에 따라 매니퓰레이터 화면은 Must 단계에서 읽기 전용이다.
// 복구(gripper open/close/home)는 관리자 인증으로 게이트되며 복구 모드
// 패널이 잠금 해제됐을 때만 보인다.
export default function ManipulatorStatusPage() {
  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
      <GripperStateCard />
      <PickDropStatus />
      <RecoveryModePanel />
    </div>
  );
}
