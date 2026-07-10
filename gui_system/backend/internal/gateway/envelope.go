package gateway

// Envelope은 WebSocket 위의 모든 메시지를 감싼다. 설계 §3-8에 따라
// 초기 구현은 단일 커넥션에 `type` 필드로 dispatch하며, per-channel
// 멀티플렉싱은 후순위로 미룬다.
type Envelope struct {
	Type      string      `json:"type"`
	RobotID   string      `json:"robot_id,omitempty"`
	Timestamp int64       `json:"timestamp"`
	Seq       uint64      `json:"seq"`
	RequestID string      `json:"request_id,omitempty"`
	Version   int         `json:"version"`
	Payload   interface{} `json:"payload,omitempty"`
}

const (
	TypeRobotPose     = "robot_pose"
	TypeRobotState    = "robot_state"
	TypeMissionState  = "mission_state"
	TypePathData      = "path_data"
	TypeScanPoints    = "scan_points"
	TypeLocalPath     = "local_path"
	TypePlannedPath   = "planned_path"
	TypeSystemLog     = "system_log"
	TypeAlert         = "alert"
	TypeETAUpdate     = "eta_update"
	TypeCommandResult = "command_result"
	TypeTelemetry     = "telemetry"
	// TypeClientLog은 한 클라이언트의 UI/명령/pose 로그 항목을 연결된
	// 다른 모든 클라이언트로 미러링해 PC + 폰이 하나의 정렬된 이벤트
	// 로그를 공유하게 한다. 원본 클라이언트는 `payload.client_id`로
	// 자기 항목을 걸러낸다.
	TypeClientLog = "client_log"
	// TypePoseEstimate는 브릿지가 수락한 2D Pose Estimate를 echo한다.
	// 수신 측은 마커를 잠시 렌더해 모든 운영자가 무엇이 설정됐는지
	// 볼 수 있다.
	TypePoseEstimate = "pose_estimate"
)
