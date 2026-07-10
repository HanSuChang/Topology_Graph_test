package data_analytics

import (
	"context"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/database"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// Estimator는 미션의 예측 ETA를 계산한다. 공식(설계 §7-2 / §7-3)은:
//
//     ETA = Σ travel_time(edge) + Σ pick_time + Σ drop_time
//           + obstacle_correction
//           + swarm_correction
//
// Cold-Start 폴백(아직 샘플 없음):
//   - 엣지별: maps/nodes.yaml의 Edge.expected_time 사용.
//   - 동작별: config의 default_pick_seconds / default_drop_seconds.
// Swarm correction은 0에서 시작한다(설계 §7-2). 데이터가 쌓이면 후속
// PR에서 군집 vs 단일 모드 기록으로부터 도출할 수 있다.
type Estimator struct {
	missions               *database.MissionRepository
	edges                  map[string]domain.Edge // key = "from->to"
	SwarmCorrectionSeconds float64
	MinSamples             int
	DefaultPickSeconds     float64
	DefaultDropSeconds     float64
}

func NewEstimator(missions *database.MissionRepository, edges []domain.Edge, swarmCorrection float64, minSamples int, pick, drop float64) *Estimator {
	m := make(map[string]domain.Edge, len(edges)*2)
	for _, e := range edges {
		m[e.FromNode+"->"+e.ToNode] = e
		if e.Direction == domain.EdgeTwoWay {
			m[e.ToNode+"->"+e.FromNode] = e
		}
	}
	if minSamples <= 0 {
		minSamples = 5
	}
	if pick <= 0 {
		pick = 8
	}
	if drop <= 0 {
		drop = 5
	}
	return &Estimator{
		missions:               missions,
		edges:                  m,
		SwarmCorrectionSeconds: swarmCorrection,
		MinSamples:             minSamples,
		DefaultPickSeconds:     pick,
		DefaultDropSeconds:     drop,
	}
}

// Predict는 예측 ETA를 초 단위로 계산한다. 단순 배송 미션에서는 tasks가
// nil일 수 있으며, 그 경우 엣지 합산만 사용한다.
func (e *Estimator) Predict(ctx context.Context, waypoints []string, tasks []domain.Task, swarm bool) float64 {
	total := 0.0
	for i := 0; i < len(waypoints)-1; i++ {
		total += e.edgeTime(ctx, waypoints[i], waypoints[i+1])
	}
	for _, t := range tasks {
		switch t.Action {
		case domain.TaskPick:
			total += e.DefaultPickSeconds
		case domain.TaskDrop:
			total += e.DefaultDropSeconds
		}
	}
	if swarm {
		total += e.SwarmCorrectionSeconds
	}
	return total
}

func (e *Estimator) edgeTime(ctx context.Context, from, to string) float64 {
	edge, hasEdge := e.edges[from+"->"+to]
	avg, samples, err := e.missions.AverageTravel(ctx, from, to)
	if err == nil && samples >= e.MinSamples {
		return avg
	}
	if err == nil && samples > 0 && hasEdge {
		// 설계 §7-3에 따라 혼합: expected_time + 관측 평균
		return (edge.ExpectedTime + avg) / 2
	}
	if hasEdge {
		return edge.ExpectedTime
	}
	return 10 // 설정된 엣지도 샘플도 없을 때의 정적 폴백
}
