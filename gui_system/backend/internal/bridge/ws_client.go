package bridge

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"log/slog"
	"net/url"
	"sync"
	"sync/atomic"
	"time"

	"github.com/gorilla/websocket"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// WSClient은 운영자가 `bridge.type: websocket`을 Python ROS2 브릿지에
// 연결할 때 쓰는 WebSocket 기반 브릿지 트랜스포트다. 하나의 TCP
// 커넥션이 다음을 운반한다:
//   - 수신 envelope: robot_pose, mission_state, alert, telemetry,
//     path_data, costmap (채널별 sink로 dispatch).
//   - 송신 command envelope: request_id로 command_result 응답과 짝지음.
//
// 클라이언트는 자체 재연결 루프를 유지해 일시적 브릿지 크래시가
// Go 측 health 플래그를 retry backoff보다 오래 끌어내리지 않게 한다.
type WSClient struct {
	address string

	connected atomic.Bool

	poseCh      chan domain.RobotState
	missionCh   chan domain.StatusEvent
	statusCh    chan domain.StatusEvent
	telemetryCh   chan domain.TelemetryData
	scanCh        chan domain.ScanPoints
	localPathCh   chan domain.LocalPath
	plannedPathCh chan domain.PlannedPath

	mu           sync.Mutex
	conn         *websocket.Conn
	pending      map[string]chan domain.CommandResult
	state        domain.SystemState
	logger       *slog.Logger
	cancelCtx    context.CancelFunc
	writeQueue   chan []byte
	reconnectMin time.Duration
	reconnectMax time.Duration
}

// NewWSClient은 "localhost:9090" 또는 "ws://host:port/path" 같은
// bridge.address를 사용 가능한 클라이언트로 해석한다. 기본값: ws://localhost:9090.
func NewWSClient(address string) *WSClient {
	return &WSClient{
		address:      address,
		poseCh:       make(chan domain.RobotState, 64),
		missionCh:    make(chan domain.StatusEvent, 64),
		statusCh:     make(chan domain.StatusEvent, 64),
		telemetryCh:  make(chan domain.TelemetryData, 128),
		scanCh:       make(chan domain.ScanPoints, 16),
		localPathCh:  make(chan domain.LocalPath, 16),
		plannedPathCh: make(chan domain.PlannedPath, 16),
		pending:      make(map[string]chan domain.CommandResult),
		writeQueue:   make(chan []byte, 64),
		reconnectMin: 500 * time.Millisecond,
		reconnectMax: 5 * time.Second,
		logger:       slog.Default().With(slog.String("module", "bridge.ws")),
	}
}

// resolveURL은 사용자가 준 address를 유효한 ws:// URL로 바꾼다.
// host:port 형태에는 ws:// scheme을 붙이고, 절대 URL은 그대로 보존해
// 보안 설정(wss://)도 동작하게 한다.
func (w *WSClient) resolveURL() (string, error) {
	addr := w.address
	if addr == "" {
		addr = "localhost:9090"
	}
	if u, err := url.Parse(addr); err == nil && (u.Scheme == "ws" || u.Scheme == "wss") {
		return addr, nil
	}
	return "ws://" + addr, nil
}

// Start는 read/write/reconnect goroutine을 띄우고 즉시 반환한다.
// 부팅 시 연결 실패는 치명적이지 않다 — 루프가 재시도하는 동안
// 대시보드는 `bridge: disconnected`를 표시한다.
func (w *WSClient) Start(ctx context.Context) error {
	c, cancel := context.WithCancel(ctx)
	w.cancelCtx = cancel
	go w.connectLoop(c)
	return nil
}

// Stop은 재연결 루프를 끝내고 현재 커넥션이 있으면 닫는다.
func (w *WSClient) Stop() {
	if w.cancelCtx != nil {
		w.cancelCtx()
	}
	w.mu.Lock()
	if w.conn != nil {
		_ = w.conn.Close()
		w.conn = nil
	}
	w.mu.Unlock()
	w.connected.Store(false)
}

func (w *WSClient) Connected() bool { return w.connected.Load() }

func (w *WSClient) SubscribePose() <-chan domain.RobotState          { return w.poseCh }
func (w *WSClient) SubscribeMissionState() <-chan domain.StatusEvent { return w.missionCh }
func (w *WSClient) SubscribeStatusEvent() <-chan domain.StatusEvent  { return w.statusCh }
func (w *WSClient) SubscribeTelemetry() <-chan domain.TelemetryData  { return w.telemetryCh }
func (w *WSClient) SubscribeScan() <-chan domain.ScanPoints          { return w.scanCh }
func (w *WSClient) SubscribeLocalPath() <-chan domain.LocalPath      { return w.localPathCh }
func (w *WSClient) SubscribePlannedPath() <-chan domain.PlannedPath  { return w.plannedPathCh }

// SendCommand는 command envelope을 직렬화하고 request_id 키로 pending
// 채널을 등록한 뒤, 브릿지가 응답하거나 context가 만료될 때까지 블록한다.
// 기본 상한은 2초이며, 브릿지가 끊겼을 때는 fast-fail해서 reader 없는
// 큐를 기다리지 않고 대시보드의 busy-button 래치를 즉시 푼다.
func (w *WSClient) SendCommand(ctx context.Context, cmd domain.Command) (domain.CommandResult, error) {
	if cmd.RequestID == "" {
		return domain.CommandResult{}, errors.New("ws bridge: empty request_id")
	}
	if !w.connected.Load() {
		return domain.CommandResult{}, fmt.Errorf("ws bridge: not connected")
	}
	envelope := map[string]interface{}{
		"type":       "command",
		"kind":       string(cmd.Kind),
		"request_id": cmd.RequestID,
		"payload":    cmd.Payload,
		"timestamp":  time.Now().UnixMilli(),
		"version":    1,
	}
	data, err := json.Marshal(envelope)
	if err != nil {
		return domain.CommandResult{}, err
	}
	respCh := make(chan domain.CommandResult, 1)
	w.mu.Lock()
	w.pending[cmd.RequestID] = respCh
	w.mu.Unlock()
	defer func() {
		w.mu.Lock()
		delete(w.pending, cmd.RequestID)
		w.mu.Unlock()
	}()

	deadline, cancel := context.WithTimeout(ctx, 2*time.Second)
	defer cancel()

	select {
	case w.writeQueue <- data:
	case <-deadline.Done():
		return domain.CommandResult{}, fmt.Errorf("ws bridge: write queue full")
	}

	select {
	case r := <-respCh:
		return r, nil
	case <-deadline.Done():
		return domain.CommandResult{}, fmt.Errorf("ws bridge: command %s timed out", cmd.RequestID)
	}
}

// GetCurrentState는 로컬에 캐시된 SystemState를 반환한다. 실시간
// reconciliation은 gateway cache로 수행되며, 이 메서드는 구독 없이
// 스냅샷이 필요한 경로용 fallback이다.
func (w *WSClient) GetCurrentState(_ context.Context) (domain.SystemState, error) {
	w.mu.Lock()
	defer w.mu.Unlock()
	cp := w.state
	cp.Robots = append([]domain.RobotState(nil), w.state.Robots...)
	return cp, nil
}

// connectLoop은 장기 실행 supervisor다. 실패 시 상한이 있는 지수
// backoff로 연결을 재수립하고, ctx가 취소될 때까지 수신 메시지를
// dispatch한다.
func (w *WSClient) connectLoop(ctx context.Context) {
	backoff := w.reconnectMin
	for {
		if ctx.Err() != nil {
			return
		}
		u, _ := w.resolveURL()
		w.logger.Info("connecting", slog.String("url", u))
		conn, _, err := websocket.DefaultDialer.DialContext(ctx, u, nil)
		if err != nil {
			w.logger.Warn("connect failed", slog.String("err", err.Error()))
			select {
			case <-ctx.Done():
				return
			case <-time.After(backoff):
			}
			if backoff < w.reconnectMax {
				backoff *= 2
				if backoff > w.reconnectMax {
					backoff = w.reconnectMax
				}
			}
			continue
		}
		w.mu.Lock()
		w.conn = conn
		w.mu.Unlock()
		w.connected.Store(true)
		w.logger.Info("connected", slog.String("url", u))
		backoff = w.reconnectMin

		// 커넥션 스코프 context. read/write 중 한쪽이 끝나면 다른 쪽도 함께
		// 정리한다. write 루프는 보낼 데이터가 없으면 죽은 소켓을 감지하지
		// 못하므로(WriteMessage 미호출), read 실패만으로는 멈추지 않는다.
		// read가 반환되면 connCancel로 write의 select를 깨우고 conn을 닫아,
		// `<-writeDone`이 영원히 블록되어 재연결·health 복구가 막히던 문제를
		// 없앤다.
		connCtx, connCancel := context.WithCancel(ctx)
		writeDone := make(chan struct{})
		go w.writeLoop(connCtx, conn, writeDone)
		w.readLoop(connCtx, conn)
		connCancel()
		_ = conn.Close()
		<-writeDone

		w.connected.Store(false)
		w.mu.Lock()
		w.conn = nil
		// 브릿지가 끊긴 동안 로봇 상태는 알 수 없으므로 캐시를 비운다.
		// 그러지 않으면 마지막 pose가 유령 로봇으로 /state/robots에 남는다.
		// 재연결 후 robot_pose가 다시 흐르면 자연히 다시 채워진다. (mission
		// 상태는 reconciliation 위해 의도적으로 유지 — dispatch의 mission_state
		// 미러링 주석 참고.)
		w.state.Robots = nil
		w.mu.Unlock()
		w.logger.Warn("disconnected, will retry")
		select {
		case <-ctx.Done():
			return
		case <-time.After(w.reconnectMin):
		}
	}
}

func (w *WSClient) writeLoop(ctx context.Context, conn *websocket.Conn, done chan<- struct{}) {
	defer close(done)
	for {
		select {
		case <-ctx.Done():
			return
		case data, ok := <-w.writeQueue:
			if !ok {
				return
			}
			_ = conn.SetWriteDeadline(time.Now().Add(2 * time.Second))
			if err := conn.WriteMessage(websocket.TextMessage, data); err != nil {
				w.logger.Warn("write failed", slog.String("err", err.Error()))
				// 소켓을 닫아 readLoop의 블록된 ReadMessage도 깨운다(half-open
				// 연결에서 read가 무한 대기하지 않도록).
				_ = conn.Close()
				return
			}
		}
	}
}

func (w *WSClient) readLoop(ctx context.Context, conn *websocket.Conn) {
	for {
		if ctx.Err() != nil {
			return
		}
		_, data, err := conn.ReadMessage()
		if err != nil {
			w.logger.Warn("read failed", slog.String("err", err.Error()))
			return
		}
		w.dispatch(data)
	}
}

// envelope은 Python 브릿지가 보내는 wire 레벨 형태다. Payload는 `type`에
// 따라 한 번 더 디코딩되어 각 도메인 채널이 타입이 있는 struct를 받는다.
type envelope struct {
	Type      string          `json:"type"`
	RobotID   string          `json:"robot_id"`
	Timestamp int64           `json:"timestamp"`
	Seq       uint64          `json:"seq"`
	RequestID string          `json:"request_id"`
	Payload   json.RawMessage `json:"payload"`
	Version   int             `json:"version"`
}

func (w *WSClient) dispatch(raw []byte) {
	var env envelope
	if err := json.Unmarshal(raw, &env); err != nil {
		w.logger.Warn("decode envelope", slog.String("err", err.Error()))
		return
	}
	switch env.Type {
	case "robot_pose":
		var rs domain.RobotState
		if err := json.Unmarshal(env.Payload, &rs); err == nil {
			if rs.RobotID == "" {
				rs.RobotID = env.RobotID
			}
			if rs.LastUpdate.IsZero() {
				rs.LastUpdate = time.Now()
			}
			select {
			case w.poseCh <- rs:
			default:
			}
			w.touchRobot(rs)
		}
	case "mission_state":
		ev := decodeStatus(env)
		select {
		case w.missionCh <- ev:
		default:
		}
		// 캐시된 SystemState에 미러링해 다음 이벤트를 기다리지 않고도
		// 재접속 후 /state/current가 최신이 되게 한다.
		w.mu.Lock()
		if s, ok := ev.Payload["status"].(string); ok {
			w.state.MissionStatus = domain.MissionStatus(s)
		}
		w.mu.Unlock()
	case "alert", "status":
		ev := decodeStatus(env)
		select {
		case w.statusCh <- ev:
		default:
		}
	case "telemetry":
		// 브릿지는 timestamp를 epoch millis(정수)로 보낸다. domain.TelemetryData의
		// Timestamp는 time.Time이라 그대로 Unmarshal하면 실패해 telemetry가 통째로
		// 드롭된다 — wire용 struct로 받아 millis를 time.Time으로 변환한다.
		var tw struct {
			RobotID   string  `json:"robot_id"`
			Velocity  float64 `json:"velocity"`
			Battery   float64 `json:"battery"`
			Timestamp int64   `json:"timestamp"`
		}
		if err := json.Unmarshal(env.Payload, &tw); err == nil {
			td := domain.TelemetryData{RobotID: tw.RobotID, Velocity: tw.Velocity, Battery: tw.Battery}
			if td.RobotID == "" {
				td.RobotID = env.RobotID
			}
			if tw.Timestamp > 0 {
				td.Timestamp = time.UnixMilli(tw.Timestamp)
			} else {
				td.Timestamp = time.Now()
			}
			select {
			case w.telemetryCh <- td:
			default:
			}
		}
	case "scan_points":
		var sp domain.ScanPoints
		if err := json.Unmarshal(env.Payload, &sp); err == nil {
			if sp.RobotID == "" {
				sp.RobotID = env.RobotID
			}
			select {
			case w.scanCh <- sp:
			default:
			}
		}
	case "local_path":
		var lp domain.LocalPath
		if err := json.Unmarshal(env.Payload, &lp); err == nil {
			if lp.RobotID == "" {
				lp.RobotID = env.RobotID
			}
			select {
			case w.localPathCh <- lp:
			default:
			}
		}
	case "planned_path":
		var pp domain.PlannedPath
		if err := json.Unmarshal(env.Payload, &pp); err == nil {
			if pp.RobotID == "" {
				pp.RobotID = env.RobotID
			}
			select {
			case w.plannedPathCh <- pp:
			default:
			}
		}
	case "command_result":
		var cr domain.CommandResult
		if err := json.Unmarshal(env.Payload, &cr); err == nil {
			if cr.RequestID == "" {
				cr.RequestID = env.RequestID
			}
			w.mu.Lock()
			ch, ok := w.pending[cr.RequestID]
			w.mu.Unlock()
			if ok {
				select {
				case ch <- cr:
				default:
				}
			}
		}
	default:
		// 알 수 없는 envelope은 조용히 통과시킨다. 브릿지가 백엔드가 아직
		// 모르는 forward-compatible 타입을 발행할 수 있기 때문이다.
	}
}

func (w *WSClient) touchRobot(rs domain.RobotState) {
	w.mu.Lock()
	defer w.mu.Unlock()
	for i := range w.state.Robots {
		if w.state.Robots[i].RobotID == rs.RobotID {
			w.state.Robots[i] = rs
			return
		}
	}
	w.state.Robots = append(w.state.Robots, rs)
}

func decodeStatus(env envelope) domain.StatusEvent {
	var ev domain.StatusEvent
	if len(env.Payload) > 0 {
		_ = json.Unmarshal(env.Payload, &ev)
	}
	if ev.EventType == "" {
		ev.EventType = env.Type
	}
	if ev.RobotID == "" {
		ev.RobotID = env.RobotID
	}
	if ev.Timestamp.IsZero() {
		ev.Timestamp = time.Now()
	}
	if ev.Payload == nil {
		var payload map[string]interface{}
		_ = json.Unmarshal(env.Payload, &payload)
		ev.Payload = payload
	}
	return ev
}
