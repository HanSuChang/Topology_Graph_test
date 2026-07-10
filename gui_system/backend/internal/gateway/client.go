package gateway

import (
	"log/slog"
	"sync"
	"sync/atomic"

	"github.com/gorilla/websocket"
)

// Client는 연결된 브라우저 하나다. 각자 두 개의 송신 채널을 갖는다 —
// pose/telemetry/path/costmap용 `latest`(size 1, 오래된 것 드랍)와
// 미션 상태/알림용 `critical`(큰 버퍼, 드랍 금지). 이 분리는 설계
// §3-8 backpressure 정책을 따른다: 부하 시 시각화 데이터는 희생해도
// 되지만 상태 전이는 잃으면 안 된다.
type Client struct {
	conn      *websocket.Conn
	latest    chan Envelope
	critical  chan Envelope
	closed    atomic.Bool
	closeOnce sync.Once
	logger    *slog.Logger
}

func newClient(conn *websocket.Conn, logger *slog.Logger) *Client {
	return &Client{
		conn:     conn,
		latest:   make(chan Envelope, 1),
		critical: make(chan Envelope, 64),
		logger:   logger,
	}
}

func (c *Client) close() {
	c.closeOnce.Do(func() {
		c.closed.Store(true)
		_ = c.conn.Close()
		close(c.latest)
		close(c.critical)
	})
}

// pushLatest는 설계 §3-8 latest-wins 드랍 정책을 구현한다: 클라이언트별
// size-1 버퍼가 가득 차면 대기 중 메시지를 버리고 새 메시지를 넣는다.
// 소비자가 최신 샘플만 필요로 하므로 pose/telemetry/path/costmap은
// 의도적으로 드랍을 허용한다.
func (c *Client) pushLatest(e Envelope) {
	select {
	case c.latest <- e:
		return
	default:
	}
	select {
	case <-c.latest:
	default:
	}
	select {
	case c.latest <- e:
	default:
	}
}

// pushCritical은 mission_state, alert, command_result용 드랍 금지 경로다.
// 버퍼가 가득 차면 클라이언트를 닫는다 — 재접속 + /state/current 호출을
// 유발해, 상태 변경을 조용히 잃는 것보다 일관성을 지킨다.
func (c *Client) pushCritical(e Envelope) bool {
	select {
	case c.critical <- e:
		return true
	default:
		c.logger.Warn("ws critical buffer full, closing client")
		c.close()
		return false
	}
}
