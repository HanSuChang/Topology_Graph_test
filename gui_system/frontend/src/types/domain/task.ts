export type TaskAction = "move" | "pick" | "drop" | "wait";
export type TaskStatus = "pending" | "running" | "completed" | "failed" | "skipped";

export interface Task {
  task_id: string;
  mission_id: string;
  node_id: string;
  action: TaskAction;
  target?: string;
  status: TaskStatus;
}
