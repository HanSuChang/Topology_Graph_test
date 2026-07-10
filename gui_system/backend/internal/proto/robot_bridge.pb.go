// Package proto는 정식 `proto/robot_bridge.proto`로부터
// `scripts/generate_proto.sh`가 생성하는 Go 바인딩의 목적지다. protoc
// 설치에 빌드가 의존하지 않도록 파일을 체크인해 두며, .proto 편집 후
// 재생성한다.
//
// gRPC 브릿지 트랜스포트가 구현되기 전(Should 단계)까지 이 파일은
// 의도적으로 패키지 마커일 뿐이다 — bridge.GRPCClient stub이 "not
// implemented"를 반환하므로 아직 어떤 proto 타입도 import되지 않는다.
package proto
