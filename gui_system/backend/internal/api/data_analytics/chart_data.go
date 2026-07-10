package data_analytics

import (
	"net/http"

	"github.com/gin-gonic/gin"
)

type chartPoint struct {
	Ts    int64   `json:"ts"`
	Value float64 `json:"value"`
}

// ChartData는 속도 / 미션 시간 / 군집 오차 차트의 시계열을 준비한다.
//   - speed: telemetry 테이블의 속도(m/s) 시계열
//   - mission_time: 완료 미션의 actual_eta(초)를 시작 시각 순으로
//   - formation_error: 팔로워 추종 오차 데이터원이 아직 없어 빈 배열
func (h *Handlers) ChartData(c *gin.Context) {
	ctx := c.Request.Context()

	speed := make([]chartPoint, 0)
	if h.Telemetry != nil {
		if pts, err := h.Telemetry.RecentSpeed(ctx, 60); err == nil {
			for _, p := range pts {
				speed = append(speed, chartPoint{Ts: p.Ts, Value: p.Value})
			}
		}
	}

	missionTime := make([]chartPoint, 0)
	if h.Missions != nil {
		if rows, err := h.Missions.Recent(ctx, 50); err == nil {
			// Recent는 최신순 → 시간 오름차순으로 뒤집어 완료 미션을 점으로.
			for i := len(rows) - 1; i >= 0; i-- {
				r := rows[i]
				if r.ActualETA != nil && r.StartTime != nil {
					missionTime = append(missionTime, chartPoint{Ts: r.StartTime.UnixMilli(), Value: *r.ActualETA})
				}
			}
		}
	}

	c.JSON(http.StatusOK, gin.H{
		"speed":           speed,
		"mission_time":    missionTime,
		"formation_error": make([]chartPoint, 0),
	})
}
