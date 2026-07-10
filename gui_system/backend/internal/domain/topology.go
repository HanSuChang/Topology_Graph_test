package domain

// ─── Node ────────────────────────────────────────────────────────────

type NodeType string

const (
	NodeStorage  NodeType = "storage"
	NodeHome     NodeType = "home"
	NodeDelivery NodeType = "delivery"
	NodeCharging NodeType = "charging"

	// Topology_Graph 통합 노드 타입(maps/nodes.yaml 동기화 산출물).
	// NodeType은 string 기반이라 미정의 값도 통과하지만, 실제 토폴로지가
	// 쓰는 타입을 상수로 명시해 프론트 NODE_COLOR/목적지 필터와 의미를 맞춘다.
	NodeLoadingZone       NodeType = "loading_zone"              // 상차 구역(목적지)
	NodeIntersection      NodeType = "intersection"              // 교차로(경유)
	NodeAreaEntry         NodeType = "area_entry"                // 구역 진입(경유)
	NodeStationingSlot    NodeType = "stationing_slot"           // 리더 정차 슬롯(목적지)
	NodeStationingSlotPre NodeType = "stationing_slot_precision" // 정밀 정차 보조
	NodeCharger           NodeType = "charger"                   // 충전 도킹(목적지)
	NodeChargerEntry      NodeType = "charger_entry"             // 충전 진입(경유)
	NodeStandby           NodeType = "standby"                   // 대기 노드
)

type Node struct {
	ID      string   `json:"node_id" yaml:"id"`
	Name    string   `json:"name" yaml:"name"`
	X       float64  `json:"x" yaml:"x"`
	Y       float64  `json:"y" yaml:"y"`
	Theta   float64  `json:"theta" yaml:"theta"`
	Type    NodeType `json:"type" yaml:"type"`
	Enabled bool     `json:"enabled" yaml:"enabled"`
}

// ─── Edge ────────────────────────────────────────────────────────────

type EdgeDirection string

const (
	EdgeOneWay EdgeDirection = "one_way"
	EdgeTwoWay EdgeDirection = "two_way"
)

type Edge struct {
	ID           string        `json:"edge_id" yaml:"id"`
	FromNode     string        `json:"from_node" yaml:"from_node"`
	ToNode       string        `json:"to_node" yaml:"to_node"`
	Distance     float64       `json:"distance" yaml:"distance"`
	ExpectedTime float64       `json:"expected_time" yaml:"expected_time"`
	Direction    EdgeDirection `json:"direction" yaml:"direction"`
	Weight       float64       `json:"weight" yaml:"weight"`
}
