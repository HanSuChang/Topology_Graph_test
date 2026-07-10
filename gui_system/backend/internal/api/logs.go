package api

import (
	"bufio"
	"encoding/json"
	"net/http"
	"os"
	"strings"

	"github.com/gin-gonic/gin"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/audit"
)

// Logs는 구조화된 감사 로그의 tail을 제공해, 대시보드의 로그 페이지가
// SQLite 텔레메트리 테이블을 건드리지 않고 최근 명령을 보여줄 수 있게
// 한다. 설계 §2에 따라 DB 장애 시에도 인가 이력을 잃으면 안 되므로
// 감사에는 JSONL 파일을 우선한다.
func (h *Handlers) Logs(c *gin.Context) {
	path := c.DefaultQuery("path", "./logs/audit.jsonl")
	f, err := os.Open(path)
	if err != nil {
		c.JSON(http.StatusOK, []audit.Entry{})
		return
	}
	defer f.Close()

	lines := tailLines(f, 200)
	out := make([]audit.Entry, 0, len(lines))
	for _, line := range lines {
		var e audit.Entry
		if err := json.Unmarshal([]byte(line), &e); err == nil {
			out = append(out, e)
		}
	}
	c.JSON(http.StatusOK, out)
}

// ClientLog는 한 대시보드 클라이언트의 UI/명령 로그 항목 하나를 연결된
// 다른 모든 클라이언트로 미러링한다. 원본 클라이언트는 `client_id`로 자기
// 항목을 걸러낸다. 인증 없음 — 명령 채널이 아니라 UX 편의 기능이며
// payload는 많아야 수백 바이트다.
func (h *Handlers) ClientLog(c *gin.Context) {
	var body struct {
		Level    string `json:"level"`
		Source   string `json:"source"`
		Message  string `json:"message"`
		ClientID string `json:"client_id"`
		Ts       int64  `json:"ts,omitempty"`
	}
	if err := c.ShouldBindJSON(&body); err != nil {
		c.Status(http.StatusBadRequest)
		return
	}
	if h.Hub != nil {
		h.Hub.BroadcastEnvelope("client_log", body)
	}
	c.Status(http.StatusNoContent)
}

// tailLines는 아주 작은 ring buffer reader다. SQLite 기반 로그 쿼리는
// Should로 미루며, 감사 JSONL은 충분히 작아 Must 단계에서는 매 요청마다
// 파일 전체를 스캔해도 무방하다.
func tailLines(f *os.File, n int) []string {
	scanner := bufio.NewScanner(f)
	scanner.Buffer(make([]byte, 1<<20), 1<<20)
	ring := make([]string, 0, n)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}
		if len(ring) == n {
			ring = ring[1:]
		}
		ring = append(ring, line)
	}
	return ring
}

