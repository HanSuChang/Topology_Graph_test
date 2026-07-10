// Package api는 React 대시보드가 소비하는 REST 엔드포인트를 묶는다.
// Handlers 리시버의 메서드는 기능별로 파일(mission.go, status.go,
// logs.go, settings.go)에 나뉘어 있어 각 파일이 설계 문서의 한 섹션에
// 대응한다.
package api

import (
	"sync"
	"time"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/api/data_analytics"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/audit"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/bridge"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/core"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/database"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/gateway"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/missionrunner"
)

// Handlers는 모든 API 핸들러가 필요로 하는 의존성을 연결한다.
// cmd/gui_main에서 구체 값 하나를 만들어 모든 메서드 리시버가 공유한다.
type Handlers struct {
	Auth      *core.AuthManager
	Audit     *audit.Logger
	Bridge    bridge.BridgeClient
	Cache     *gateway.Cache
	Idem      *gateway.IdempotencyCache
	DB        *database.DB
	Missions  *database.MissionRepository
	Estimator *data_analytics.Estimator
	Runner    *missionrunner.Manager

	Nodes  []domain.Node
	Edges  []domain.Edge
	Robots []domain.Robot

	// Hub은 명령 핸들러가 연결된 모든 대시보드 클라이언트로 ad-hoc
	// 이벤트를 fan-out하게 한다 — PC와 폰이 상태를 공유하도록 client_log
	// 미러링과 pose_estimate echo에 사용된다.
	Hub *gateway.Hub

	// 진행 중 미션 추적 — 시작 시 missions 행을 만들고, 완료/초기화/재시작
	// 시 actual_eta(경과 시간)와 함께 닫는다. 미션 이력·ETA 차트의 데이터원.
	missionMu       sync.Mutex
	activeMissionID string
	missionStart    time.Time
}
