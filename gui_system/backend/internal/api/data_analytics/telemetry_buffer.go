package data_analytics

// BatchWriter 타입 자체는 내부 커넥션을 공유할 수 있도록 internal/database에
// 있다. 이 파일은 telemetry-buffer 개념을 data_analytics 패키지 표면에서
// 닿을 수 있게 하기 위해 존재한다 — 예: 향후 on-demand flush 엔드포인트나
// 버퍼 깊이를 health 지표로 노출하는 엔드포인트.
//
// 설계 §7-4에 따라 writer는 15초마다 또는 100행마다 flush한다. 종단 간
// 배선(subscribe → push → flush)은 cmd/gui_main에서 설정된다.
