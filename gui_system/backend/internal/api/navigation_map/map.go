// Package navigation_map은 대시보드의 네비게이션 맵 모듈이 소비하는
// map / node / edge / path API를 제공한다. 설계 §6에 따라 대시보드는
// SLAM 맵과 등록된 토폴로지만 렌더하며 자유 좌표 입력은 허용하지 않으므로,
// 이 패키지의 역할은 운영자가 선택 가능한 것만 정확히 노출하는 것이다.
package navigation_map

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// Handlers는 시작 시 maps/nodes.yaml에서 한 번 로드한 in-memory 토폴로지와
// 로봇 레지스트리를 보유한다. 토폴로지가 정적이므로 대시보드는 맵 페이지를
// 열 때 /state/map을 한 번만 폴링하고 이후로는 하지 않는다.
type Handlers struct {
	Nodes   []domain.Node
	Edges   []domain.Edge
	Robots  []domain.Robot
	MapYAML string
}

func NewHandlers(nodes []domain.Node, edges []domain.Edge, robots []domain.Robot, mapYAML string) *Handlers {
	return &Handlers{Nodes: nodes, Edges: edges, Robots: robots, MapYAML: mapYAML}
}

// Map은 프론트엔드의 MapRenderer.setTopology가 쓰는 토폴로지 + 로봇
// 결합 payload를 반환한다.
func (h *Handlers) Map(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{
		"nodes":  h.Nodes,
		"edges":  h.Edges,
		"robots": h.Robots,
	})
}
