package bridge

import (
	"encoding/json"
	"fmt"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// JSONAdapter는 Python 브릿지에서 오는 raw WebSocket 프레임을 도메인
// 타입으로 변환한다. wire 스키마는 브라우저 쪽 gateway가 쓰는 envelope과
// 일치해 같은 형태가 양쪽 hop을 흐른다. 이 struct가 향후 JSON 레벨
// 변경(이름 변경, 필드 추가)으로부터 백엔드 나머지를 격리한다.
type JSONAdapter struct{}

func NewJSONAdapter() *JSONAdapter { return &JSONAdapter{} }

type wireEnvelope struct {
	Type    string          `json:"type"`
	Payload json.RawMessage `json:"payload"`
}

func (JSONAdapter) Decode(raw []byte) (string, json.RawMessage, error) {
	var env wireEnvelope
	if err := json.Unmarshal(raw, &env); err != nil {
		return "", nil, fmt.Errorf("json envelope: %w", err)
	}
	return env.Type, env.Payload, nil
}

func (JSONAdapter) DecodePose(raw json.RawMessage) (domain.RobotState, error) {
	var rs domain.RobotState
	err := json.Unmarshal(raw, &rs)
	return rs, err
}

func (JSONAdapter) DecodeMissionState(raw json.RawMessage) (domain.StatusEvent, error) {
	var ev domain.StatusEvent
	err := json.Unmarshal(raw, &ev)
	return ev, err
}

// EncodeCommand는 WebSocket 브릿지로 보낼 Command를 직렬화한다.
func (JSONAdapter) EncodeCommand(cmd domain.Command) ([]byte, error) {
	return json.Marshal(cmd)
}
