package database

import (
	"context"
	"database/sql"
	"sort"
	"sync"
	"time"
)

// Row는 버퍼링된 단일 텔레메트리 샘플이다. 설계 §7-4에 따라 하나씩
// 기록하지 않고, BatchWriter가 15초마다 또는 100행마다(먼저 도달하는
// 쪽) flush한다.
type Row struct {
	MissionID string
	RobotID   string
	Timestamp time.Time
	PoseX     float64
	PoseY     float64
	Velocity  float64
	Battery   float64
}

// TelemetryRepository는 텔레메트리 샘플을 영속화한다. 배칭이 영속화
// 계약의 일부이므로(호출 측은 개별 INSERT를 보지 않음) BatchWriter
// goroutine도 소유한다.
type TelemetryRepository struct {
	Repository
}

func NewTelemetryRepository(d *DB) *TelemetryRepository {
	return &TelemetryRepository{Repository: NewRepository(d)}
}

// BatchWriter는 Row를 버퍼링했다가 15초 tick 또는 100행 누적 시점에
// 트랜잭션으로 flush한다. 하나의 리포지토리에 바인딩된다.
type BatchWriter struct {
	repo    *TelemetryRepository
	mu      sync.Mutex
	buf     []Row
	maxSize int
	maxAge  time.Duration
	stop    chan struct{}
	stopped chan struct{}
}

func NewBatchWriter(repo *TelemetryRepository) *BatchWriter {
	return &BatchWriter{
		repo:    repo,
		maxSize: 100,
		maxAge:  15 * time.Second,
		stop:    make(chan struct{}),
		stopped: make(chan struct{}),
	}
}

func (b *BatchWriter) Push(r Row) {
	b.mu.Lock()
	b.buf = append(b.buf, r)
	full := len(b.buf) >= b.maxSize
	b.mu.Unlock()
	if full {
		_ = b.flush(context.Background())
	}
}

func (b *BatchWriter) Run(ctx context.Context) {
	t := time.NewTicker(b.maxAge)
	defer t.Stop()
	defer close(b.stopped)
	for {
		select {
		case <-ctx.Done():
			_ = b.flush(context.Background())
			return
		case <-b.stop:
			_ = b.flush(context.Background())
			return
		case <-t.C:
			_ = b.flush(ctx)
		}
	}
}

func (b *BatchWriter) Stop() {
	close(b.stop)
	<-b.stopped
}

func (b *BatchWriter) flush(ctx context.Context) error {
	b.mu.Lock()
	if len(b.buf) == 0 {
		b.mu.Unlock()
		return nil
	}
	batch := b.buf
	b.buf = nil
	b.mu.Unlock()

	conn := b.repo.db.conn
	tx, err := conn.BeginTx(ctx, nil)
	if err != nil {
		return err
	}
	stmt, err := tx.PrepareContext(ctx,
		`INSERT INTO telemetry (mission_id, robot_id, timestamp, pose_x, pose_y, velocity, battery)
		 VALUES (?, ?, ?, ?, ?, ?, ?)`)
	if err != nil {
		_ = tx.Rollback()
		return err
	}
	defer stmt.Close()
	for _, r := range batch {
		if _, err := stmt.ExecContext(ctx, r.MissionID, r.RobotID, r.Timestamp, r.PoseX, r.PoseY, r.Velocity, r.Battery); err != nil {
			_ = tx.Rollback()
			return err
		}
	}
	return tx.Commit()
}

// SpeedPoint는 속도 시계열의 한 점이다(ts=epoch millis, value=m/s).
// 분석 탭 속도 차트의 TimeSeriesPoint와 wire 형태를 맞춘다.
type SpeedPoint struct {
	Ts    int64   `json:"ts"`
	Value float64 `json:"value"`
}

// RecentSpeed는 최근 telemetry를 1초 버킷으로 평균낸 속도 시계열을 시간
// 오름차순으로 반환한다(buckets = 반환할 최근 초 수). 여러 로봇의 5Hz 샘플을
// 초당 평균 한 점으로 모아, 차트가 톱니처럼 난잡해지지 않고 매끄러운 군집
// 평균 속도를 보여준다.
func (r *TelemetryRepository) RecentSpeed(ctx context.Context, buckets int) ([]SpeedPoint, error) {
	if buckets <= 0 {
		buckets = 60
	}
	// 초당 ~수십 샘플(로봇 수 × 레이트)을 덮도록 충분히 가져온다.
	rows, err := r.db.conn.QueryContext(ctx,
		`SELECT timestamp, velocity FROM telemetry WHERE velocity IS NOT NULL ORDER BY timestamp DESC LIMIT ?`, buckets*40)
	if err != nil {
		return nil, err
	}
	defer rows.Close()
	type agg struct {
		sum float64
		n   int
	}
	m := map[int64]*agg{}
	for rows.Next() {
		var ts time.Time
		var v sql.NullFloat64
		if err := rows.Scan(&ts, &v); err != nil {
			return nil, err
		}
		if !v.Valid {
			continue
		}
		b := ts.Unix() // 1초 버킷
		a := m[b]
		if a == nil {
			a = &agg{}
			m[b] = a
		}
		a.sum += v.Float64
		a.n++
	}
	if err := rows.Err(); err != nil {
		return nil, err
	}
	keys := make([]int64, 0, len(m))
	for k := range m {
		keys = append(keys, k)
	}
	sort.Slice(keys, func(i, j int) bool { return keys[i] < keys[j] })
	if len(keys) > buckets {
		keys = keys[len(keys)-buckets:]
	}
	out := make([]SpeedPoint, 0, len(keys))
	for _, k := range keys {
		a := m[k]
		out = append(out, SpeedPoint{Ts: k * 1000, Value: a.sum / float64(a.n)})
	}
	return out, nil
}
