package gateway

import (
	"log/slog"
	"net/http"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
)

var upgrader = websocket.Upgrader{
	CheckOrigin:     func(r *http.Request) bool { return true },
	ReadBufferSize:  1024,
	WriteBufferSize: 4096,
}

// Hub은 연결된 모든 브라우저로 메시지를 fan-out한다. 클라이언트
// 레지스트리와 fanout.go·API 핸들러가 쓰는 두 broadcast 경로
// (BroadcastLatest / BroadcastCritical)를 소유한다.
type Hub struct {
	mu      sync.RWMutex
	clients map[*Client]struct{}
	logger  *slog.Logger
}

func NewHub(logger *slog.Logger) *Hub {
	return &Hub{clients: map[*Client]struct{}{}, logger: logger}
}

func (h *Hub) add(c *Client) {
	h.mu.Lock()
	h.clients[c] = struct{}{}
	h.mu.Unlock()
}

func (h *Hub) remove(c *Client) {
	h.mu.Lock()
	delete(h.clients, c)
	h.mu.Unlock()
	c.close()
}

// BroadcastLatest는 pose / telemetry / path / costmap용이다.
// 느린 클라이언트는 producer를 막는 대신 이전 메시지를 드랍한다.
func (h *Hub) BroadcastLatest(e Envelope) {
	h.mu.RLock()
	defer h.mu.RUnlock()
	for c := range h.clients {
		if c.closed.Load() {
			continue
		}
		c.pushLatest(e)
	}
}

// BroadcastCritical은 mission_state / alert / command_result용이다.
// 버퍼가 밀린 클라이언트는 닫아서 재접속·reconciliation하게 한다.
func (h *Hub) BroadcastCritical(e Envelope) {
	h.mu.RLock()
	defer h.mu.RUnlock()
	for c := range h.clients {
		if c.closed.Load() {
			continue
		}
		c.pushCritical(e)
	}
}

// BroadcastEnvelope은 ad-hoc 이벤트(client_log 미러, pose_estimate echo)를
// fan-out해야 하는 API 핸들러가 쓰는 공개 단발성 broadcast다. critical로
// 취급해 한 뷰어가 느려도 다중 클라이언트 로그 순서가 유지된다.
func (h *Hub) BroadcastEnvelope(typ string, payload interface{}) {
	h.BroadcastCritical(Envelope{
		Type:      typ,
		Timestamp: time.Now().UnixMilli(),
		Version:   1,
		Payload:   payload,
	})
}

// writePump은 두 채널을 WebSocket으로 직렬화한다. 상태 변경 순서를
// 지키기 위해 critical을 먼저 비우고, NAT/keepalive 타임아웃을 견디도록
// 20초 ping을 보낸다.
func (h *Hub) writePump(c *Client) {
	pingTicker := time.NewTicker(20 * time.Second)
	defer pingTicker.Stop()
	for {
		select {
		case e, ok := <-c.critical:
			if !ok {
				return
			}
			if err := c.conn.WriteJSON(e); err != nil {
				h.remove(c)
				return
			}
		case e, ok := <-c.latest:
			if !ok {
				return
			}
			if err := c.conn.WriteJSON(e); err != nil {
				h.remove(c)
				return
			}
		case <-pingTicker.C:
			if err := c.conn.WriteControl(websocket.PingMessage, nil, time.Now().Add(5*time.Second)); err != nil {
				h.remove(c)
				return
			}
		}
	}
}

// HandleWS는 HTTP 요청을 WebSocket으로 업그레이드하고 새 Client를 Hub에
// 등록한다. 프로토콜이 server-push 전용이라 reader goroutine은 최소한이다
// — 명령은 REST로 흐르므로 Must 경로에서 브라우저는 WS 프레임을 실제로
// 보내지 않는다.
func (h *Hub) HandleWS(c *gin.Context) {
	conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		h.logger.Warn("ws upgrade failed", slog.String("err", err.Error()))
		return
	}
	cl := newClient(conn, h.logger)
	h.add(cl)
	go h.writePump(cl)
	go func() {
		defer h.remove(cl)
		for {
			if _, _, err := conn.NextReader(); err != nil {
				return
			}
		}
	}()
}
