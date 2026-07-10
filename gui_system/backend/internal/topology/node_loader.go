package topology

import (
	"fmt"
	"os"

	"gopkg.in/yaml.v3"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// File은 maps/nodes.yaml의 on-disk 형태다. 스키마는 도메인 타입과 동일하며
// — yaml 태그가 struct에 중복돼 있어 이 로더는 단순 unmarshal로 유지된다.
type File struct {
	Nodes []domain.Node `yaml:"nodes"`
	Edges []domain.Edge `yaml:"edges"`
}

// Load는 maps/nodes.yaml을 토폴로지로 파싱한다. 이 파일은 시작 시
// 권위 있는 것으로 취급된다: 설계 §6-2가 임의 좌표를 금지하므로,
// 대시보드가 이후 Goal로 제시하는 모든 노드는 여기서 비롯돼야 한다.
func Load(path string) (*File, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("read nodes file %s: %w", path, err)
	}
	var n File
	if err := yaml.Unmarshal(data, &n); err != nil {
		return nil, fmt.Errorf("parse nodes file: %w", err)
	}
	return &n, nil
}
