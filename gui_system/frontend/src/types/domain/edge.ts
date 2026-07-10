export type EdgeDirection = "one_way" | "two_way";

export interface Edge {
  edge_id: string;
  from_node: string;
  to_node: string;
  distance: number;
  expected_time: number;
  direction: EdgeDirection;
  weight: number;
}
