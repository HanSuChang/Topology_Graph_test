import { useRobotState } from "@/hooks/useRobotState";
import { FormationErrorCard, FollowerStatusCard, SwarmRelationView } from "./components";

// 설계 §5-2에 따라 군집 화면은 관찰용이다. 이 페이지는 3대 로봇을 기존
// leader / follower_1 / follower_2 슬롯에 연결한다.
export default function SwarmStatusPage() {
  const robots = useRobotState();
  const leader = robots.find((r) => r.robot_id === "tb3_leader");
  const f1 = robots.find((r) => r.robot_id === "follower_1");
  const f2 = robots.find((r) => r.robot_id === "follower_2");
  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
      <FormationErrorCard leader={leader} f1={f1} f2={f2} />
      <FollowerStatusCard f1={f1} f2={f2} />
      <SwarmRelationView />
    </div>
  );
}
