package api

import (
	"context"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"
)

// StateCurrent은 대시보드가 접속/재접속 시 쓰는 전체 reconciliation
// 스냅샷을 제공한다. 브릿지가 닿으면 최신 브릿지 조회를 우선해 응답이
// 권위 있는 ROS2 상태를 반영하게 하고, 브릿지 타임아웃 시 로컬 cache로
// 폴백한다.
func (h *Handlers) StateCurrent(c *gin.Context) {
	ctx, cancel := context.WithTimeout(c.Request.Context(), 2*time.Second)
	defer cancel()
	// 미션 컨텍스트(상태/목적지/ETA/긴급)는 백엔드가 명령으로 추적하는
	// Cache가 권위다. 브릿지 상태(로봇 라이브)에 이 필드들을 덮어써, 브릿지가
	// mission_state를 발행하지 않아도 요약 카드가 채워진다.
	snap := h.Cache.Snapshot()
	if s, err := h.Bridge.GetCurrentState(ctx); err == nil {
		s.MissionStatus = snap.MissionStatus
		s.CurrentGoalNode = snap.CurrentGoalNode
		s.ETASeconds = snap.ETASeconds
		s.EmergencyActive = snap.EmergencyActive
		s.Route = snap.Route
		c.JSON(http.StatusOK, s)
		return
	}
	c.JSON(http.StatusOK, snap)
}

// StateSystem은 설계 §3-10의 분할 변형이다: 무거운 로봇 리스트 없이
// 미션 / 긴급 / 버튼 활성 상태만 제공한다.
func (h *Handlers) StateSystem(c *gin.Context) {
	snap := h.Cache.Snapshot()
	c.JSON(http.StatusOK, gin.H{
		"mission_status":     snap.MissionStatus,
		"emergency_active":   snap.EmergencyActive,
		"current_mission_id": snap.CurrentMissionID,
		"current_goal_node":  snap.CurrentGoalNode,
		"eta_seconds":        snap.ETASeconds,
	})
}

// StateRobots는 로봇별 스냅샷만 반환한다. 미션 컨텍스트가 필요 없는
// 로봇 / 군집 / 매니퓰레이터 상태 페이지가 사용한다.
func (h *Handlers) StateRobots(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{"robots": h.Cache.RobotsSnapshot()})
}
