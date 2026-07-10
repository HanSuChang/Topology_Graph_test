package bridge

import (
	"context"
	"fmt"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// New는 config로부터 BridgeClient 구현체를 선택한다. 설계 §1에 따라
// 선택은 config 기반(build-tag 아님)이라 같은 바이너리가 재컴파일 없이
// 트랜스포트를 교체할 수 있다.
//
// 브릿지 라이프사이클과 미션 컨텍스트는 분리되어 있다 — 브릿지가 안 떠
// 있어도 백엔드는 운영자 명령으로 미션 의도(route/goal/ETA)를 추적·broadcast
// 하므로 노드 필터·요약 카드는 봇 없이도 즉시 동작한다(README §7.12).
func New(kind, address string) (BridgeClient, error) {
	switch kind {
	case "grpc":
		return NewGRPCClient(address), nil
	case "", "websocket", "ws":
		return NewWSClient(address), nil
	default:
		return nil, fmt.Errorf("unknown bridge.type %q", kind)
	}
}

// BridgeClient은 Go 백엔드와 Python ROS2 Bridge 사이의 트랜스포트
// 비의존 인터페이스다. 구현체는 gRPC나 WebSocket을 쓸 수 있으며, 설계
// §3-5에 따라 선택은 config 시점에 이뤄져 트랜스포트가 바뀌어도
// 대시보드 / DB / 맵 코드는 바뀌지 않는다.
type BridgeClient interface {
	Start(ctx context.Context) error
	Stop()
	Connected() bool

	SubscribePose() <-chan domain.RobotState
	SubscribeMissionState() <-chan domain.StatusEvent
	SubscribeStatusEvent() <-chan domain.StatusEvent
	SubscribeTelemetry() <-chan domain.TelemetryData
	SubscribeScan() <-chan domain.ScanPoints
	SubscribeLocalPath() <-chan domain.LocalPath
	SubscribePlannedPath() <-chan domain.PlannedPath

	SendCommand(ctx context.Context, cmd domain.Command) (domain.CommandResult, error)
	GetCurrentState(ctx context.Context) (domain.SystemState, error)
}
