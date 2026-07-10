package domain

import "time"

// ─── Commands (dashboard → bridge) ───────────────────────────────────

// CommandKind은 대시보드가 보낼 수 있는 모든 변경 명령을 열거한다.
// 문자열 값은 안정적인 wire 식별자다 — 감사 로그와 WebSocket envelope에
// 나타나므로 기존 행들이 이 값에 의존한다.
type CommandKind string

const (
	CmdStartMission   CommandKind = "start_mission"
	CmdPause          CommandKind = "pause"
	CmdResume         CommandKind = "resume"
	CmdResetMission   CommandKind = "reset_mission"
	CmdChangeGoal     CommandKind = "change_goal"
	CmdEmergencyStop  CommandKind = "emergency_stop"
	CmdEmergencyClear CommandKind = "emergency_clear"
	// CmdSetPoseEstimate은 RViz 스타일 2D Pose Estimate forward다.
	// Payload: {x, y, theta, robot_id?}. 브릿지가 PoseWithCovarianceStamped를
	// /initialpose에 발행해 AMCL이 재시드할 수 있게 한다.
	CmdSetPoseEstimate CommandKind = "set_pose_estimate"
)

// Command은 모든 변경 REST 호출이 브릿지로 넘기기 전에 감싸는 envelope다.
// RequestID가 idempotency cache를 구동한다.
type Command struct {
	RequestID string                 `json:"request_id"`
	Kind      CommandKind            `json:"kind"`
	Payload   map[string]interface{} `json:"payload,omitempty"`
}

type CommandResult struct {
	RequestID string                 `json:"request_id"`
	Accepted  bool                   `json:"accepted"`
	Message   string                 `json:"message,omitempty"`
	Data      map[string]interface{} `json:"data,omitempty"`
}

// ─── System snapshot (returned by /state/current) ────────────────────

// SystemState는 대시보드 재접속 후 /state/current가 반환하는 reconciliation
// 스냅샷이다. 대시보드는 "지금 세계 상태는?"에 대한 모든 답을 이 하나의
// 형태에서 도출한다.
type SystemState struct {
	MissionStatus    MissionStatus `json:"mission_status"`
	EmergencyActive  bool          `json:"emergency_active"`
	Robots           []RobotState  `json:"robots"`
	CurrentMissionID string        `json:"current_mission_id,omitempty"`
	CurrentGoalNode  string        `json:"current_goal_node,omitempty"`
	ETASeconds       float64       `json:"eta_seconds,omitempty"`
	// Route는 현재 미션의 토폴로지 경로 노드 id 순서다(start→…→goal). 대시보드
	// 맵이 이 노드만 표시하고, 리더 진행에 따라 노드 라이트를 켠다.
	Route []string `json:"route,omitempty"`
}

// ─── Asynchronous events (bridge → dashboard) ────────────────────────

// StatusEvent는 gateway WS로 `mission_state` 또는 `alert`로 push되는
// 범용 "무언가 일어남" payload다. 미션 상태 변경, pick/drop 전이, 팔로워
// 연결 끊김이 모두 StatusEvent로 흐른다.
type StatusEvent struct {
	EventType string                 `json:"event_type"`
	RobotID   string                 `json:"robot_id,omitempty"`
	MissionID string                 `json:"mission_id,omitempty"`
	Timestamp time.Time              `json:"timestamp"`
	Severity  string                 `json:"severity,omitempty"`
	Payload   map[string]interface{} `json:"payload,omitempty"`
}

// NodeGoal은 대시보드가 Goal을 선택할 때 브릿지로 전송된다. `change_goal`
// 명령 payload를 반영하면서, 활성 target을 강조하기 위해 네비게이션 맵
// 으로도 forward된다.
type NodeGoal struct {
	RobotID      string `json:"robot_id"`
	TargetNodeID string `json:"target_node_id"`
	MissionID    string `json:"mission_id,omitempty"`
}

type PathPoint struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// ScanPoints는 라이다 스캔 한 사이클을 map 프레임의 점 리스트로 운반한다.
// 브릿지가 /scan을 TF로 변환해 보내고 프런트가 맵 위에 점 클라우드로 그린다.
type ScanPoints struct {
	RobotID string      `json:"robot_id"`
	Points  []PathPoint `json:"points"`
}

// LocalPath는 주행로직(C++ DWA local planner)이 동적 장애물을 만났을 때
// 선택한 우회 호(arc)를 map 프레임 점 리스트로 운반한다. 토폴로지 route
// (mission.go가 path_data로 emit)와 의미가 다르므로 envelope 타입을 분리.
// 빈 Points는 회피 종료를 의미해, 프런트의 옛 호 선이 자연 소거된다.
type LocalPath struct {
	RobotID string      `json:"robot_id"`
	Points  []PathPoint `json:"points"`
}

// PlannedPath는 주행로직(C++)이 자체 산출한 전역 계획 경로를 map 프레임 점
// 리스트로 운반한다. 현재 backend mission.go의 로컬 Dijkstra가 path_data로
// 보내는 것과 의미가 같지만 소스가 다르다 — C++ 측 publisher가 활성화되면
// 프런트는 같은 global 키에 latest-wins로 덮어써져 C++ 경로가 표시된다.
// 빈 Points는 미션 종료/취소를 의미해 점선이 자연 소거된다.
type PlannedPath struct {
	RobotID string      `json:"robot_id"`
	Points  []PathPoint `json:"points"`
}

type TelemetryData struct {
	RobotID   string    `json:"robot_id"`
	Timestamp time.Time `json:"timestamp"`
	Velocity  float64   `json:"velocity"`
	Battery   float64   `json:"battery"`
}

