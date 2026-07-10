export type RobotType = "leader" | "follower" | "manipulator";

export interface Robot {
  robot_id: string;
  type: RobotType;
  name: string;
  capabilities: string[];
  color: string;
  enabled: boolean;
}
