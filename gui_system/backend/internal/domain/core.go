// Package domain은 모든 하위 레이어(API 핸들러, gateway, 브릿지 어댑터,
// SQLite 리포지토리)가 다루는 트랜스포트 비의존 값 타입을 보유한다.
// 설계 §3에 따라 이 struct들은 트랜스포트 세부(protobuf, JSON 래퍼)를
// 절대 갖지 않으며, 어댑터가 입출력을 변환해 wire 필드 이름 변경이
// 비즈니스 로직으로 번지지 않게 한다.
//
// 파일 구성:
//   - core.go      — robots, missions, tasks (움직이는 것)
//   - topology.go  — nodes, edges (갈 수 있는 곳)
//   - command.go   — commands, results, system state, status events
package domain

import "time"

// ─── Robot ────────────────────────────────────────────────────────────

type RobotType string

const (
	RobotLeader      RobotType = "leader"
	RobotFollower    RobotType = "follower"
	RobotManipulator RobotType = "manipulator"
)

type Robot struct {
	ID           string    `json:"robot_id" yaml:"id"`
	Type         RobotType `json:"type" yaml:"type"`
	Name         string    `json:"name" yaml:"name"`
	Capabilities []string  `json:"capabilities" yaml:"capabilities"`
	Color        string    `json:"color" yaml:"color"`
	Enabled      bool      `json:"enabled" yaml:"enabled"`
}

// ─── Robot state ─────────────────────────────────────────────────────

type ConnectionState string

const (
	ConnOnline  ConnectionState = "online"
	ConnOffline ConnectionState = "offline"
	ConnLost    ConnectionState = "lost"
)

type RobotStatus string

const (
	StatusIdle    RobotStatus = "idle"
	StatusMoving  RobotStatus = "moving"
	StatusPicking RobotStatus = "picking"
	StatusError   RobotStatus = "error"
	StatusStopped RobotStatus = "stopped"

	// TurtleBot 리더가 RC카(팔로워)에 발행하는 상태 토픽 값. 대문자 표기는
	// ROS2 토픽 계약을 그대로 따른다(NORMAL/SLOW/...). 리더는 이 상태로
	// 팔로워의 정지/출발/복귀를 지시하고, 대시보드는 한글 뱃지로 표시한다.
	StatusNormal        RobotStatus = "NORMAL"         // 정상 주행
	StatusSlow          RobotStatus = "SLOW"           // 감속
	StatusStop          RobotStatus = "STOP"           // 정지 지시
	StatusAvoiding      RobotStatus = "AVOIDING"       // 장애물 회피 중
	StatusStationing    RobotStatus = "STATIONING"     // 구역 정차 중
	StatusUnloading     RobotStatus = "UNLOADING"      // 하차 작업 중
	StatusReturnAllowed RobotStatus = "RETURN_ALLOWED" // 복귀 허가
	StatusReturning     RobotStatus = "RETURNING"      // 단독 복귀 중
)

type Pose struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Theta float64 `json:"theta"`
}

type RobotState struct {
	RobotID         string          `json:"robot_id"`
	Pose            Pose            `json:"pose"`
	Battery         float64         `json:"battery"`
	Status          RobotStatus     `json:"status"`
	CurrentNode     string          `json:"current_node"`
	ConnectionState ConnectionState `json:"connection_state"`
	LastUpdate      time.Time       `json:"last_update"`
}

// ─── Mission ─────────────────────────────────────────────────────────

type MissionType string

const (
	MissionDelivery MissionType = "delivery"
	MissionPatrol   MissionType = "patrol"
	MissionTest     MissionType = "test"
)

type MissionStatus string

const (
	MissionPending   MissionStatus = "pending"
	MissionRunning   MissionStatus = "running"
	MissionPaused    MissionStatus = "paused"
	MissionCompleted MissionStatus = "completed"
	MissionFailed    MissionStatus = "failed"
	MissionAborted   MissionStatus = "aborted"

	// 물류 이송 미션 FSM 6단계(설계 §2 전체 미션 흐름). 리더(TurtleBot)가
	// 진행하는 단계로, 대시보드 요약 카드가 한글 라벨로 현재 단계를 보여준다.
	//   Loading → Formation Driving → Area Stationing →
	//   Sequential Unloading → Leader Return → Mission Queue Loop
	MissionLoading             MissionStatus = "loading"              // 상차
	MissionFormationDriving    MissionStatus = "formation_driving"    // 대열 주행
	MissionAreaStationing      MissionStatus = "area_stationing"      // 구역 정차
	MissionSequentialUnloading MissionStatus = "sequential_unloading" // 순차 하차
	MissionLeaderReturn        MissionStatus = "leader_return"        // 리더 복귀
	MissionQueue               MissionStatus = "mission_queue"        // 미션 큐 대기/순환
)

type Mission struct {
	ID               string        `json:"id"`
	Type             MissionType   `json:"type"`
	ItemID           string        `json:"item_id"`
	Waypoints        []string      `json:"waypoints"`
	AssignedRobots   []string      `json:"assigned_robots"`
	StartNode        string        `json:"start_node"`
	TargetNode       string        `json:"target_node"`
	Priority         int           `json:"priority"`
	Status           MissionStatus `json:"status"`
	CurrentTaskIndex int           `json:"current_task_index"`
	StartTime        *time.Time    `json:"start_time,omitempty"`
	EndTime          *time.Time    `json:"end_time,omitempty"`
}

// ─── Task ────────────────────────────────────────────────────────────

type TaskAction string

const (
	TaskMove TaskAction = "move"
	TaskPick TaskAction = "pick"
	TaskDrop TaskAction = "drop"
	TaskWait TaskAction = "wait"
)

type TaskStatus string

const (
	TaskPending   TaskStatus = "pending"
	TaskRunning   TaskStatus = "running"
	TaskCompleted TaskStatus = "completed"
	TaskFailed    TaskStatus = "failed"
	TaskSkipped   TaskStatus = "skipped"
)

type Task struct {
	TaskID    string     `json:"task_id"`
	MissionID string     `json:"mission_id"`
	NodeID    string     `json:"node_id"`
	Action    TaskAction `json:"action"`
	Target    string     `json:"target"`
	Status    TaskStatus `json:"status"`
}
