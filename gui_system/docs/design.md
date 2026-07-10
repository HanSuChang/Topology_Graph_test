# GUI 시스템 설계 문서

본 문서는 설계 단계에서 합의된 GUI 시스템 구조의 스냅샷이다.
실행 가능한 구현 계획은 `../docs/plan.md` 또는 `~/.claude/plans/`에 저장된 계획 파일을 참조한다.

요약:

- `gui_main`: 전체 시스템 진입점 (조립자)
- Core Server: Gin 기반 REST/WS/정적 서빙, Audit Log(JSONL), Emergency Stop
- Robot Interface: BridgeClient 인터페이스로 gRPC/WebSocket 전환 가능, mock 모드 지원
- Dashboard UI: React PWA, Navigation Map + Data & Analytics 하위 모듈
- 데이터 모델: Robot / RobotState / Node / Edge / Mission / Task — 모두 `robot_id` 기준 분리
- ETA: 노드별 평균 + pick/drop 평균 + 장애물 보정 + 군집 보정(초기 0)
- WebSocket: 단일 연결 + type 기반 dispatch, Pose/Telemetry/Costmap은 latest wins
- gRPC ↔ WebSocket 전환 체크포인트: 3단계 착수 시점부터 주차별 점검

자세한 내용은 회의록에 정리된 설계 본문을 참조한다.
