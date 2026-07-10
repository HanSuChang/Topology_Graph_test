package bridge

import "github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"

// ProtoAdapter는 protobuf wire 타입을 트랜스포트 비의존 도메인 모델로
// 변환해, 생성된 `.pb.go` 타입을 백엔드 나머지로부터 격리한다. gRPC
// 브릿지가 미구현인 동안은 stub이며, `internal/proto/robot_bridge.pb.go`가
// 생성되면 JSONAdapter와 같은 형태로 구체 메서드가 들어온다.
//
// 설계 §3-2에 따라 계약은 한 방향뿐이다: pb → domain. 송신 명령은
// 의존성 표면을 작게 유지하기 위해 별도 encoder를 거친다.
type ProtoAdapter struct{}

func NewProtoAdapter() *ProtoAdapter { return &ProtoAdapter{} }

// PoseFromProto는 입력 도메인 값을 그대로 반환하는 placeholder다.
// gRPC가 연결되면 생성된 pb.RobotPose 타입을 받아 필드명을 변환한다.
func (ProtoAdapter) PoseFromProto(rs domain.RobotState) domain.RobotState { return rs }
