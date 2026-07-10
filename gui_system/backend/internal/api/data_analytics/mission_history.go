package data_analytics

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/database"
)

// MissionHistory는 GET /api/v1/missions를 뒷받침하며, 예측 vs 실제 ETA를
// 담은 최근 50개 미션을 반환한다. 대시보드의 미션 이력 테이블과 예측 vs
// 실제 Recharts가 이 단일 엔드포인트에서 데이터를 받는다.
func (h *Handlers) MissionHistory(c *gin.Context) {
	if h.Missions == nil {
		c.JSON(http.StatusOK, []database.HistoryRow{})
		return
	}
	rows, err := h.Missions.Recent(c.Request.Context(), 50)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	}
	c.JSON(http.StatusOK, rows)
}
