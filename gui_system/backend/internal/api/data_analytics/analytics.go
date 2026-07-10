// Package data_analytics는 React 대시보드에 제공되는 미션 이력 / ETA /
// 텔레메트리 표면을 담당한다. ETA 예측은 이 서브트리 밖의 무언가를
// import하지 않고도 mission 리포지토리의 이동시간 샘플에 접근할 수 있도록
// core/가 아닌 여기에 둔다.
package data_analytics

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/database"
)

// Handlers는 미션 이력, 텔레메트리 버퍼, 차트 엔드포인트가 공유하는
// 영속화 + estimator를 담는다.
type Handlers struct {
	Missions  *database.MissionRepository
	Telemetry *database.TelemetryRepository
	Estimator *Estimator
}

func NewHandlers(missions *database.MissionRepository, telemetry *database.TelemetryRepository, estimator *Estimator) *Handlers {
	return &Handlers{Missions: missions, Telemetry: telemetry, Estimator: estimator}
}

// Analytics는 대시보드 Overview KPI 카드가 소비하는 최신 숫자 스냅샷을
// 반환하는 작은 dispatcher 엔드포인트다.
func (h *Handlers) Analytics(c *gin.Context) {
	count := 0
	avg := 0.0
	if h.Missions != nil {
		if rows, err := h.Missions.Recent(c.Request.Context(), 50); err == nil {
			count = len(rows)
			sum, n := 0.0, 0
			for _, r := range rows {
				if r.ActualETA != nil {
					sum += *r.ActualETA
					n++
				}
			}
			if n > 0 {
				avg = sum / float64(n)
			}
		}
	}
	c.JSON(http.StatusOK, gin.H{
		"recent_mission_count": count,
		"avg_eta_seconds":      avg,
	})
}
