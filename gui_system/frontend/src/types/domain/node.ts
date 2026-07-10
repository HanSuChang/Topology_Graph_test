export type NodeType = "storage" | "home" | "delivery" | "charging";

export interface MapNode {
  node_id: string;
  name: string;
  x: number;
  y: number;
  theta: number;
  type: NodeType;
  enabled: boolean;
}
