package core

import (
	"net/http"
	"sync/atomic"

	"github.com/gin-gonic/gin"
)

type HealthState struct {
	// BridgeProbe는 브릿지의 라이브 연결 상태를 조회한다. 연결은 비동기
	// 재연결 루프에서 수시로 바뀌므로 부팅 시 스냅샷이 아니라 요청마다
	// 호출해야 한다. nil이면 disconnected로 본다.
	BridgeProbe func() bool
	DBReady     atomic.Bool
}

func (h *HealthState) Handler() gin.HandlerFunc {
	return func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"server": "ok",
			"bridge": connStr(h.BridgeProbe != nil && h.BridgeProbe()),
			"db":     dbStr(h.DBReady.Load()),
		})
	}
}

func connStr(b bool) string {
	if b {
		return "connected"
	}
	return "disconnected"
}

func dbStr(b bool) string {
	if b {
		return "ready"
	}
	return "unready"
}
