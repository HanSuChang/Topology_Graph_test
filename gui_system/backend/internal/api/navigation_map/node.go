package navigation_map

import (
	"net/http"

	"github.com/gin-gonic/gin"
)

// Nodes_는 노드 리스트만 반환한다 — 전체 엣지 그래프가 필요 없는 미션
// 제어 화면의 드롭다운에 유용하다.
func (h *Handlers) Nodes_(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{"nodes": h.Nodes})
}

// SelectNode은 향후 "미리보기 + 확인" UX를 위해 예약돼 있다. Must
// 단계에서는 대시보드가 선택한 target으로 /missions/start를 단순 POST한다.
func (h *Handlers) SelectNode(c *gin.Context) {
	var body struct {
		NodeID string `json:"node_id"`
	}
	if err := c.ShouldBindJSON(&body); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "bad body"})
		return
	}
	for _, n := range h.Nodes {
		if n.ID == body.NodeID {
			c.JSON(http.StatusOK, n)
			return
		}
	}
	c.JSON(http.StatusNotFound, gin.H{"error": "node not registered"})
}
