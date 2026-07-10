package bridge

import (
	"context"
	"errors"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// GRPCClient은 gRPC 기반 브릿지 트랜스포트의 placeholder다. config에서
// `bridge.type: grpc`로 데모를 빌드할 수 있도록 factory에 연결돼 있지만,
// 실제 proto 교환은 설계 §11에 따라 Should 단계에서 구현된다.
type GRPCClient struct {
	address string
}

func NewGRPCClient(address string) *GRPCClient { return &GRPCClient{address: address} }

func (g *GRPCClient) Start(context.Context) error { return errors.New("grpc bridge not implemented") }
func (g *GRPCClient) Stop()                       {}
func (g *GRPCClient) Connected() bool             { return false }

func (g *GRPCClient) SubscribePose() <-chan domain.RobotState         { ch := make(chan domain.RobotState); close(ch); return ch }
func (g *GRPCClient) SubscribeMissionState() <-chan domain.StatusEvent { ch := make(chan domain.StatusEvent); close(ch); return ch }
func (g *GRPCClient) SubscribeStatusEvent() <-chan domain.StatusEvent  { ch := make(chan domain.StatusEvent); close(ch); return ch }
func (g *GRPCClient) SubscribeTelemetry() <-chan domain.TelemetryData  { ch := make(chan domain.TelemetryData); close(ch); return ch }
func (g *GRPCClient) SubscribeScan() <-chan domain.ScanPoints          { ch := make(chan domain.ScanPoints); close(ch); return ch }
func (g *GRPCClient) SubscribeLocalPath() <-chan domain.LocalPath      { ch := make(chan domain.LocalPath); close(ch); return ch }
func (g *GRPCClient) SubscribePlannedPath() <-chan domain.PlannedPath  { ch := make(chan domain.PlannedPath); close(ch); return ch }

func (g *GRPCClient) SendCommand(context.Context, domain.Command) (domain.CommandResult, error) {
	return domain.CommandResult{}, errors.New("grpc bridge not implemented")
}
func (g *GRPCClient) GetCurrentState(context.Context) (domain.SystemState, error) {
	return domain.SystemState{}, errors.New("grpc bridge not implemented")
}
