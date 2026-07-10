package navigation_map

import (
	"net/http"

	"github.com/gin-gonic/gin"
)

// Path는 Should 단계 global/local 경로 REST 표면의 stub이다. Must 데모는
// 경로를 gateway WebSocket으로 path_data envelope으로 받지만, 여기에
// 노출해 두면 대시보드가 재접속 시 가장 최근 경로를 조회할 수 있다.
func (h *Handlers) Path(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{
		"robot_id":  c.Query("robot_id"),
		"kind":      c.DefaultQuery("kind", "global"),
		"points":    []interface{}{},
		"available": false,
	})
}
