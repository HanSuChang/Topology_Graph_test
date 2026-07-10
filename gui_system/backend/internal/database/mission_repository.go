package database

import (
	"context"
	"database/sql"
	"encoding/json"
	"time"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// MissionRepository는 missions, mission_events, node_travel_times의
// 읽기/쓰기를 담당한다. 설계 §7-4에 따라 이 repo의 모든 메서드는
// "critical" 쓰기다 — 미션 시작/완료가 누락되면 이력 보고가 손상되므로
// 텔레메트리 배치 writer를 우회한다.
type MissionRepository struct {
	Repository
}

func NewMissionRepository(d *DB) *MissionRepository {
	return &MissionRepository{Repository: NewRepository(d)}
}

// Create는 미션 시작 행을 기록한다. predictedETA는 0일 수 있지만, 이력
// 조회 전에 WebSocket으로 도착하는 mission ID를 대시보드가 해석할 수
// 있도록 행은 그대로 생성된다.
func (r *MissionRepository) Create(ctx context.Context, m domain.Mission, predictedETA float64) error {
	wp, _ := json.Marshal(m.Waypoints)
	_, err := r.db.conn.ExecContext(ctx,
		`INSERT INTO missions (id, type, item_id, status, target_node, start_node, waypoints, start_time, predicted_eta)
		 VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)`,
		m.ID, string(m.Type), m.ItemID, string(m.Status), m.TargetNode, m.StartNode, string(wp), nowIf(m.StartTime), predictedETA,
	)
	return err
}

func (r *MissionRepository) Complete(ctx context.Context, missionID string, status domain.MissionStatus, actualETA float64) error {
	_, err := r.db.conn.ExecContext(ctx,
		`UPDATE missions SET status=?, end_time=?, actual_eta=? WHERE id=?`,
		string(status), time.Now().UTC(), actualETA, missionID,
	)
	return err
}

func (r *MissionRepository) AppendEvent(ctx context.Context, missionID, eventType, detail string) error {
	_, err := r.db.conn.ExecContext(ctx,
		`INSERT INTO mission_events (mission_id, timestamp, event_type, detail) VALUES (?, ?, ?, ?)`,
		missionID, time.Now().UTC(), eventType, detail,
	)
	return err
}

// HistoryRow는 /api/v1/missions로 반환되는 JSON 형태다.
type HistoryRow struct {
	ID           string     `json:"id"`
	Type         string     `json:"type"`
	Status       string     `json:"status"`
	TargetNode   string     `json:"target_node"`
	StartTime    *time.Time `json:"start_time,omitempty"`
	EndTime      *time.Time `json:"end_time,omitempty"`
	PredictedETA *float64   `json:"predicted_eta,omitempty"`
	ActualETA    *float64   `json:"actual_eta,omitempty"`
}

func (r *MissionRepository) Recent(ctx context.Context, limit int) ([]HistoryRow, error) {
	if limit <= 0 {
		limit = 50
	}
	rows, err := r.db.conn.QueryContext(ctx,
		`SELECT id, type, status, COALESCE(target_node,''), start_time, end_time, predicted_eta, actual_eta
		 FROM missions ORDER BY COALESCE(start_time, '') DESC LIMIT ?`, limit,
	)
	if err != nil {
		return nil, err
	}
	defer rows.Close()
	out := make([]HistoryRow, 0, limit)
	for rows.Next() {
		var hr HistoryRow
		var st, et sql.NullTime
		var pe, ae sql.NullFloat64
		if err := rows.Scan(&hr.ID, &hr.Type, &hr.Status, &hr.TargetNode, &st, &et, &pe, &ae); err != nil {
			return nil, err
		}
		if st.Valid {
			t := st.Time
			hr.StartTime = &t
		}
		if et.Valid {
			t := et.Time
			hr.EndTime = &t
		}
		if pe.Valid {
			v := pe.Float64
			hr.PredictedETA = &v
		}
		if ae.Valid {
			v := ae.Float64
			hr.ActualETA = &v
		}
		out = append(out, hr)
	}
	return out, rows.Err()
}

// RecordTravel은 관측된 엣지 통과 시간 하나를 저장하며, ETA estimator가
// 엣지별 평균을 정교화하는 데 사용한다.
func (r *MissionRepository) RecordTravel(ctx context.Context, fromNode, toNode string, seconds float64, swarm bool) error {
	swarmInt := 0
	if swarm {
		swarmInt = 1
	}
	_, err := r.db.conn.ExecContext(ctx,
		`INSERT INTO node_travel_times (from_node, to_node, seconds, swarm_mode, recorded_at)
		 VALUES (?, ?, ?, ?, ?)`,
		fromNode, toNode, seconds, swarmInt, time.Now().UTC(),
	)
	return err
}

// AverageTravel은 엣지의 평균 통과 시간 + 샘플 수를 반환한다.
// ETA estimator의 Cold Start 폴백에서 사용한다.
func (r *MissionRepository) AverageTravel(ctx context.Context, fromNode, toNode string) (avgSeconds float64, samples int, err error) {
	row := r.db.conn.QueryRowContext(ctx,
		`SELECT AVG(seconds), COUNT(*) FROM node_travel_times WHERE from_node=? AND to_node=?`,
		fromNode, toNode,
	)
	var avg sql.NullFloat64
	var count int
	if err := row.Scan(&avg, &count); err != nil {
		return 0, 0, err
	}
	if !avg.Valid {
		return 0, 0, nil
	}
	return avg.Float64, count, nil
}

func nowIf(t *time.Time) time.Time {
	if t != nil {
		return *t
	}
	return time.Now().UTC()
}
