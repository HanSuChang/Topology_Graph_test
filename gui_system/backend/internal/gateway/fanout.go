package gateway

import (
	"context"
	"sync/atomic"
	"time"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/bridge"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// Fanout은 브릿지 이벤트를 hub와 in-memory 상태 cache로 흘려보낸다.
// 또한 프로토콜이 사용하는 단조 증가 `seq` 필드를 부여해 클라이언트가
// critical 채널의 누락(gap)을 감지할 수 있게 한다.
type Fanout struct {
	bridge bridge.BridgeClient
	hub    *Hub
	cache  *Cache
	seq    atomic.Uint64
	// telemetrySink는 설정되면 각 텔레메트리 샘플을 추가로 받는다(예: DB
	// 배치 writer). 라이브 송출(hub)과 영속화(DB)를 한 곳에서 분기한다.
	telemetrySink func(domain.TelemetryData)
}

func NewFanout(b bridge.BridgeClient, h *Hub, c *Cache) *Fanout {
	return &Fanout{bridge: b, hub: h, cache: c}
}

// SetTelemetrySink는 텔레메트리 영속화 콜백을 연결한다. fanout.Run 전에 호출.
func (f *Fanout) SetTelemetrySink(fn func(domain.TelemetryData)) {
	f.telemetrySink = fn
}

func (f *Fanout) Run(ctx context.Context) {
	go f.runPose(ctx)
	go f.runMission(ctx)
	go f.runTelemetry(ctx)
	go f.runScan(ctx)
	go f.runLocalPath(ctx)
	go f.runPlannedPath(ctx)
	go f.runStatus(ctx)
	go f.watchConnection(ctx)
}

// watchConnection은 브릿지 연결 상태를 폴링해, connected→disconnected 전이
// 시점에 로봇 캐시를 비운다. 브릿지가 끊긴 동안 로봇 상태는 알 수 없으므로
// 마지막 pose를 유령 로봇으로 /state/robots에 남기지 않는다. 재연결 후
// robot_pose가 다시 흐르면 runPose의 ApplyRobot이 캐시를 다시 채운다.
func (f *Fanout) watchConnection(ctx context.Context) {
	ticker := time.NewTicker(time.Second)
	defer ticker.Stop()
	prev := f.bridge.Connected()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			cur := f.bridge.Connected()
			if prev && !cur {
				f.cache.ClearRobots()
			}
			prev = cur
		}
	}
}

func (f *Fanout) nextSeq() uint64 { return f.seq.Add(1) }

func (f *Fanout) env(t string, robotID string, payload interface{}) Envelope {
	return Envelope{
		Type:      t,
		RobotID:   robotID,
		Timestamp: time.Now().UnixMilli(),
		Seq:       f.nextSeq(),
		Version:   1,
		Payload:   payload,
	}
}

func (f *Fanout) runPose(ctx context.Context) {
	ch := f.bridge.SubscribePose()
	for {
		select {
		case <-ctx.Done():
			return
		case rs, ok := <-ch:
			if !ok {
				return
			}
			f.cache.ApplyRobot(rs)
			f.hub.BroadcastLatest(f.env(TypeRobotPose, rs.RobotID, rs))
		}
	}
}

func (f *Fanout) runMission(ctx context.Context) {
	ch := f.bridge.SubscribeMissionState()
	for {
		select {
		case <-ctx.Done():
			return
		case e, ok := <-ch:
			if !ok {
				return
			}
			f.hub.BroadcastCritical(f.env(TypeMissionState, "", e))
		}
	}
}

func (f *Fanout) runStatus(ctx context.Context) {
	ch := f.bridge.SubscribeStatusEvent()
	for {
		select {
		case <-ctx.Done():
			return
		case e, ok := <-ch:
			if !ok {
				return
			}
			f.hub.BroadcastCritical(f.env(TypeAlert, e.RobotID, e))
		}
	}
}

func (f *Fanout) runTelemetry(ctx context.Context) {
	ch := f.bridge.SubscribeTelemetry()
	for {
		select {
		case <-ctx.Done():
			return
		case t, ok := <-ch:
			if !ok {
				return
			}
			f.hub.BroadcastLatest(f.env(TypeTelemetry, t.RobotID, t))
			if f.telemetrySink != nil {
				f.telemetrySink(t)
			}
		}
	}
}

// runPlannedPath는 주행로직(C++)이 자체 산출한 전역 계획 경로를 latest-wins로
// fan-out한다. 활성화되면 mission.go의 로컬 Dijkstra 기반 path_data를 같은
// frontend 키(`robotId:global`)에 덮어써, 표시 경로가 주행 측 출처로 전환된다.
// 빈 Points는 미션 종료를 의미해 점선이 자연 소거된다.
func (f *Fanout) runPlannedPath(ctx context.Context) {
	ch := f.bridge.SubscribePlannedPath()
	for {
		select {
		case <-ctx.Done():
			return
		case pp, ok := <-ch:
			if !ok {
				return
			}
			f.hub.BroadcastLatest(f.env(TypePlannedPath, pp.RobotID, pp))
		}
	}
}

// runLocalPath는 주행로직(C++ DWA)이 동적 장애물을 회피하며 선택한 우회 호를
// latest-wins로 fan-out한다. 옛 호는 의미 없으니 최신만 클라이언트에 도달.
// 회피 종료 시 C++가 빈 Points를 보내면 프런트 선이 자연 소거된다.
func (f *Fanout) runLocalPath(ctx context.Context) {
	ch := f.bridge.SubscribeLocalPath()
	for {
		select {
		case <-ctx.Done():
			return
		case lp, ok := <-ch:
			if !ok {
				return
			}
			f.hub.BroadcastLatest(f.env(TypeLocalPath, lp.RobotID, lp))
		}
	}
}

// runScan은 라이다 스캔(map 프레임 점 클라우드)을 latest-wins로 fan-out한다.
// 시각화 전용이라 옛 스캔은 의미가 없어 가장 최근 한 사이클만 클라이언트에 도달.
func (f *Fanout) runScan(ctx context.Context) {
	ch := f.bridge.SubscribeScan()
	for {
		select {
		case <-ctx.Done():
			return
		case s, ok := <-ch:
			if !ok {
				return
			}
			f.hub.BroadcastLatest(f.env(TypeScanPoints, s.RobotID, s))
		}
	}
}
