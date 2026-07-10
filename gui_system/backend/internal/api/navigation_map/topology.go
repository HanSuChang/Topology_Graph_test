package navigation_map

import (
	"net/http"

	"github.com/gin-gonic/gin"
)

// Topology는 맵의 엣지 부분을 반환하며, 대시보드는 이를 노드 간 연결을
// 그리는 데 사용한다. 엣지 방향(one_way / two_way)과 weight는 설정된
// maps/nodes.yaml에서 온다.
func (h *Handlers) Topology(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{"edges": h.Edges})
}
