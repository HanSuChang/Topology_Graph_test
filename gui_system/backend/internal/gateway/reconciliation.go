package gateway

import (
	"sync"
	"time"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// Cache는 최신 권위 시스템 상태를 메모리에 보관해, 재접속한 대시보드가
// REST reconciliation 엔드포인트(`/api/v1/state/current`)로 다시 동기화될
// 수 있게 한다. 설계 §3-10에 따라 이 cache는 추후 분할된
// /state/{system,robots,map} 엔드포인트로도 제공될 수 있어, 로봇을 flat
// list가 아닌 id별로 추적한다.
type Cache struct {
	mu      sync.RWMutex
	system  domain.SystemState
	robots  map[string]domain.RobotState
	updated time.Time
}

func NewCache() *Cache {
	return &Cache{robots: map[string]domain.RobotState{}}
}

func (c *Cache) ApplyRobot(rs domain.RobotState) {
	c.mu.Lock()
	c.robots[rs.RobotID] = rs
	c.updated = time.Now()
	c.mu.Unlock()
}

// ClearRobots는 로봇 캐시를 비운다. 브릿지가 끊겨 더 이상 권위 있는 로봇
// 상태를 받을 수 없을 때 호출해, 마지막 pose가 유령 로봇으로 /state/robots에
// 남지 않게 한다. 재연결 후 robot_pose가 다시 흐르면 ApplyRobot이 다시 채운다.
func (c *Cache) ClearRobots() {
	c.mu.Lock()
	c.robots = map[string]domain.RobotState{}
	c.updated = time.Now()
	c.mu.Unlock()
}

func (c *Cache) SetSystem(s domain.SystemState) {
	c.mu.Lock()
	c.system = s
	c.updated = time.Now()
	c.mu.Unlock()
}

// SetMission은 미션 상태·목적지·ETA를 한 번에 갱신한다. 미션 FSM은 ROS2
// 쪽이지만, 운영자가 GUI에서 내린 명령(시작/목적지 변경)을 백엔드가 추적해
// 요약 카드가 즉시 반영하게 한다.
func (c *Cache) SetMission(status domain.MissionStatus, goal string, eta float64, route []string) {
	c.mu.Lock()
	c.system.MissionStatus = status
	c.system.CurrentGoalNode = goal
	c.system.ETASeconds = eta
	c.system.Route = route
	c.updated = time.Now()
	c.mu.Unlock()
}

// SetMissionStatus는 목적지/ETA는 그대로 두고 상태만 바꾼다(일시정지/재개 등).
func (c *Cache) SetMissionStatus(status domain.MissionStatus) {
	c.mu.Lock()
	c.system.MissionStatus = status
	c.updated = time.Now()
	c.mu.Unlock()
}

func (c *Cache) SetEmergency(active bool) {
	c.mu.Lock()
	c.system.EmergencyActive = active
	c.updated = time.Now()
	c.mu.Unlock()
}

func (c *Cache) Snapshot() domain.SystemState {
	c.mu.RLock()
	defer c.mu.RUnlock()
	out := c.system
	out.Robots = make([]domain.RobotState, 0, len(c.robots))
	for _, r := range c.robots {
		out.Robots = append(out.Robots, r)
	}
	return out
}

func (c *Cache) RobotsSnapshot() []domain.RobotState {
	c.mu.RLock()
	defer c.mu.RUnlock()
	out := make([]domain.RobotState, 0, len(c.robots))
	for _, r := range c.robots {
		out = append(out, r)
	}
	return out
}
