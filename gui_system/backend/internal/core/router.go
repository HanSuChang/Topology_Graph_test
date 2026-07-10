package core

import (
	"github.com/gin-gonic/gin"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/gateway"
)

// Routes는 RegisterRoutes가 소비하는 의존성 묶음이다. 호출 측
// (cmd/gui_main)이 internal/api와 그 서브패키지의 구체 핸들러로 한 번
// 구성한다.
type Routes struct {
	// mission / status / logs / settings 핸들러
	Mission interface {
		StartMission(*gin.Context)
		Pause(*gin.Context)
		Resume(*gin.Context)
		ResetMission(*gin.Context)
		ChangeGoal(*gin.Context)
		EmergencyStop(*gin.Context)
		EmergencyClear(*gin.Context)
		PoseEstimate(*gin.Context)
	}
	Status interface {
		StateCurrent(*gin.Context)
		StateSystem(*gin.Context)
		StateRobots(*gin.Context)
	}
	Logs interface {
		Logs(*gin.Context)
		ClientLog(*gin.Context)
	}
	Settings interface{ Login(*gin.Context) }

	// navigation_map
	NavMap interface {
		Map(*gin.Context)
		Nodes_(*gin.Context)
		SelectNode(*gin.Context)
		Topology(*gin.Context)
		Path(*gin.Context)
		MapInfo(*gin.Context)
		MapImage(*gin.Context)
	}

	// data_analytics
	Analytics interface {
		Analytics(*gin.Context)
		MissionHistory(*gin.Context)
		ChartData(*gin.Context)
	}
}

// RegisterRoutes는 /api/v1과 /ws 아래에 REST + WebSocket 엔드포인트를
// 바인딩한다. 설계 §2 / §5-1에 따른 구분은 다음과 같다:
//   - GET 엔드포인트(state, history, charts): 인증 없음
//   - POST 명령 엔드포인트: AuthManager.RequireAuth
//   - POST /api/v1/emergency/stop: EmergencyMiddleware (allow-list + rate limit)
func RegisterRoutes(s *Server, r Routes, hub *gateway.Hub) {
	s.engine.Use(func(c *gin.Context) {
		c.Set("auth", s.Auth)
		c.Next()
	})

	api := s.engine.Group("/api/v1")

	// 공개 읽기
	api.GET("/health", s.Health.Handler())
	api.GET("/state/current", r.Status.StateCurrent)
	api.GET("/state/system", r.Status.StateSystem)
	api.GET("/state/robots", r.Status.StateRobots)
	api.GET("/state/map", r.NavMap.Map)
	api.GET("/map/info", r.NavMap.MapInfo)
	api.GET("/map/image", r.NavMap.MapImage)

	api.GET("/missions", r.Analytics.MissionHistory)
	api.GET("/analytics", r.Analytics.Analytics)
	api.GET("/analytics/chart", r.Analytics.ChartData)

	api.GET("/nodes", r.NavMap.Nodes_)
	api.POST("/nodes/select", r.NavMap.SelectNode)
	api.GET("/topology", r.NavMap.Topology)
	api.GET("/paths", r.NavMap.Path)

	api.GET("/logs", r.Logs.Logs)
	api.POST("/client_log", r.Logs.ClientLog)

	// 2D Pose Estimate — 관리자 인증 없음(위치추정 보조 기능만).
	api.POST("/pose_estimate", r.Mission.PoseEstimate)

	// 인증 부트스트랩
	api.POST("/auth/login", r.Settings.Login)

	// 긴급 정지 — 인증 없음, 단 allow-list + rate limit 적용
	emergency := api.Group("/emergency")
	emergency.POST("/stop", EmergencyMiddleware(s.Allow, s.Limiter, s.Audit), r.Mission.EmergencyStop)

	// 인증 필요 명령 엔드포인트
	cmd := api.Group("")
	cmd.Use(s.Auth.RequireAuth())
	cmd.POST("/missions/start", r.Mission.StartMission)
	cmd.POST("/missions/pause", r.Mission.Pause)
	cmd.POST("/missions/resume", r.Mission.Resume)
	cmd.POST("/missions/reset", r.Mission.ResetMission)
	cmd.POST("/missions/goal", r.Mission.ChangeGoal)
	cmd.POST("/emergency/clear", r.Mission.EmergencyClear)

	// WebSocket gateway
	s.engine.GET("/ws", hub.HandleWS)
}
