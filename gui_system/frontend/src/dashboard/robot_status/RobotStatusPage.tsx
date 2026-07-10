import { Card } from "@/dashboard/layout/Card";
import { useRobotState } from "@/hooks/useRobotState";
import { TurtleBot3Card } from "./components";

export default function RobotStatusPage() {
  const robots = useRobotState();
  if (robots.length === 0) {
    return <Card>로봇 데이터 없음 (Mock Bridge가 5Hz 시작 전)</Card>;
  }
  return (
    <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
      {robots.map((r) => (
        <TurtleBot3Card key={r.robot_id} robot={r} />
      ))}
    </div>
  );
}
