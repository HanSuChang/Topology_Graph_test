# CLAUDE.md

스마트 물류 관제 시스템 — TurtleBot3 군집(Leader + Follower 2대) + 매니퓰레이터를
한 화면에서 감시·제어하는 운영 콘솔. 단일 페이지 대시보드 + PWA + 다중 클라이언트
(PC ↔ 폰) 실시간 동기화. ROS2 Humble 백엔드 통합.

## 런타임 3종 (모노레포)

| 런타임 | 디렉토리 | 스택 | 산출물 |
| --- | --- | --- | --- |
| Backend | `backend/` | Go 1.22 + Gin + gorilla/websocket + modernc.org/sqlite(pure Go) | `gui_main` 단일 바이너리, `:8080` |
| Frontend | `frontend/` | React 18 + Vite + TS + Tailwind + Recharts + Framer Motion + vite-plugin-pwa | 정적 dist(백엔드가 서빙), dev `:5173` |
| Bridge | `bridge/` | Python `rclpy` + `asyncio` + `websockets` | `ros2_bridge` 모듈, WS `:9090` |

데이터 흐름:
```
ROS2(tf:map→base_footprint[마커 pose 주력] + amcl_pose/odom/scan/plan/battery_state)
  → bridge(Python, ROS_DOMAIN_ID=27) --JSON/WS--> backend(Go :8080)
  → WS /ws 팬아웃 → PC 브라우저 + 폰 PWA (client_log / pose_estimate 양방향 동기화)
```

## 빌드 & 실행

Go는 PATH에 없을 수 있음 — `~/.local/go/bin`에 있고, 빌드 시 `-buildvcs=false` 필수.

```bash
# 백엔드 (빌드+실행, :8080) — 시작 시 LAN URL을 함께 로그로 출력
./scripts/run_backend.sh
# 예: INFO open in browser local=http://localhost:8080 lan=[http://192.168.0.7:8080]
# `lan` 옆 URL을 같은 LAN의 호스트 Chrome(예: 윈도우)에 그대로 붙여 접속.

# 프런트 dev 서버 (:5173)
cd frontend && npm run dev

# 프런트 프로덕션 빌드 (백엔드가 서빙할 dist 생성)
cd frontend && npx tsc -b && npm run build      # 또는 ./scripts/build_frontend.sh

# ROS2 브릿지 (:9090) — ROS_DOMAIN_ID=27 자동 export, /opt/ros/humble 자동 source
./scripts/run_bridge.sh
```

검증: `go build` / `npx tsc -b` / `npm run build` 모두 0 에러 유지. Python은 `ast.parse` OK.

## 설정 파일

- `backend/configs/config.yaml` — 단일 진실 소스. `bridge.type: websocket`, `bridge.address: localhost:9090`. 경로는 config 디렉토리 기준 상대.
  - `bridge.type`: `websocket`(Python 브릿지) | `grpc`(스텁). mock 모드는 2026-06-17에 제거됨 — 브릿지 없이도 백엔드가 미션 컨텍스트(route/goal/ETA)를 추적·broadcast하므로 노드 필터·요약 카드는 그대로 동작한다.
- `bridge/config.yaml` — ROS2 전용. `ros.domain_id: 27`, `ros.robots`(logical id ↔ 토픽 namespace 매핑). 기본 활성: `tb3_leader`(namespace 비움) + `rc_car_follower`(namespace `rc_car`, `/rc_car/odom` 구독). TurtleBot3 follower 추가 시 follower_1/follower_2 주석 해제.

## ROS2 동기화가 안 될 때 (자주 겪는 이슈)

GUI에 로봇이 안 뜨면 아래 체인을 순서대로 확인:

1. **Python 브릿지가 떠 있나** — `:9090` LISTEN? 백엔드 `/healthz`에서 `bridge: connected`?
   안 떠 있으면 ROS2 토픽이 백엔드로 전달 안 됨(가장 흔한 원인). `./scripts/run_bridge.sh`.
2. **`websockets` 파이썬 패키지 설치됨?** — 없으면 브릿지가 "websockets package not installed"
   출력 후 WS 서빙 안 함. **현재 환경은 `pip install --user 'websockets>=12'`로 16.0 설치
   완료(`~/.local/lib/python3.10/site-packages`).** apt `python3-websockets`(9.1)는 Python
   3.10에서 브릿지가 쓰는 API와 안 맞으니 새 환경에서도 apt 말고 pip로 설치할 것.
   확인: `python3 -c "import websockets; print(websockets.__version__)"`.
3. **같은 와이파이/서브넷?** — TurtleBot3와 GCS PC IP 앞 3옥텟 일치(예 둘 다 `192.168.0.x`).
   다르면 DDS 멀티캐스트 discovery 안 닿아 토픽 0개. (GCS IP는 `192.168.0.7`였음)
4. **`ROS_DOMAIN_ID=27`** 봇·PC 양쪽 일치.
5. **`ROS_LOCALHOST_ONLY`** 미설정 또는 0. `1`이면 원격 봇 안 보임.
6. **AP 격리(client isolation)** 꺼짐 — 같은 SSID여도 게스트망이면 기기 간 통신 차단.

빠른 진단: ROS2 source된 터미널에서 `export ROS_DOMAIN_ID=27 && ros2 topic list` →
`/amcl_pose`,`/odom`,`/scan`,`/plan` 보이면 네트워크 OK(브릿지만 띄우면 됨).

## 프런트엔드 구조 (리팩토링 완료)

- `dashboard/MainDashboardPage.tsx` (~100줄) — 레이아웃 + WS 로그 브릿지만. 실제 화면은
  `dashboard/cards/`로 분리: MissionControlCard, PoseEstimateCard, SummaryCard,
  RobotStatusCard, ManipulatorCard, LogsStrip, shared.tsx.
- `dashboard/navigation_map/` — MapRenderer.ts, MapPanel.tsx, MapCanvas.tsx, layers.ts,
  mapTypes.ts, coordinateTransform.ts.
- `lib/` — eventLog.ts(CLIENT_ID 기반 다중 클라이언트 로그 브로드캐스트), poseEstimate.ts,
  api.ts, websocket.ts.
- 라우트는 `App.tsx`에 둘뿐: `/`(MainDashboardPage), `/분석`(AnalyticsPage).

**주의 — 레거시(dead) 디렉토리**: `dashboard/` 아래 `overview/`, `settings/`,
`robot_status/`, `swarm_status/`, `manipulator_status/`, `mission_control/`,
`navigation_map/`(일부), `logs/`의 옛 *Page.tsx들은 단일 페이지 통합 이후
라우팅되지 않음. README 7.11에 삭제 대상으로 플래그됨. 새 작업은 `cards/`에서.
(`camera/`는 2026-06-25에 터틀봇3 주행 카메라 제거와 함께 삭제됨 — 매니퓰레이터 카메라는 `cards/ManipulatorCard.tsx`로 잔존.)

## 코드 컨벤션

- **주석은 전부 한국어.** 코드/식별자/문자열/기술용어(WebSocket, AMCL, Nav2, SLAM, DDS 등)는 영어 유지.
- WS 프로토콜: envelope 구조. latest-wins 채널(size 1) vs critical 채널(size 64), idempotency 캐시.
- 맵 파이프라인: PGM 디코드 → island 제거 → dark-bbox crop → CCW 90도 회전 → PNG, 캔버스에 stretch-fit.
  **노드·로봇·경로·클릭·pose는 이 변환을 그대로 복원해 맵 위에 정렬한다** — `/map/info`가
  origin·crop bbox·원본 dims를 노출하고 `coordinateTransform.toScreen/fromScreen`이 world→원본
  픽셀→crop→CCW90→stretch를 적용(`MapRenderer`가 `vp.slam`에 적재). 맵 미로드 시엔 단순(중앙) 변환 폴백.
- heading(로봇 chevron · pose 화살표)은 `headingToScreenAngle`로 toScreen 투영각을 써서 그린다.
  raw `cos/sin(world theta)`는 맵 회전을 무시해 방향이 틀어지므로 금지.
- 로봇 마커는 heading 방향 화살표/chevron. Nav2 global plan은 하늘색 점선.
- **마커 pose source는 TF `map→base_footprint`(20Hz, `topic_subscriber.py` `_on_tf_timer`)**. amcl_pose는
  발행이 불규칙·저빈도이고 보정 시 점프해 마커가 끊겨 보였다 — TF는 odom 빈도로 매끄럽고 map 프레임
  정합이라(주행로직 `mission_loop`과 동일 소스) 마커가 부드럽다. TF 가용 시 amcl push는 생략(충돌 방지),
  미가용 시 amcl→odom 폴백. spin 루프는 틱당 8콜백 드레인(`__main__.py`)해 /tf 고빈도에 20Hz 타이머가
  굶지 않게 한다. (`topic_filter.py`의 RateGate는 정의만 있고 미연결 — 실제 pose throttle 없음.)
- **마커는 위치·heading 모두 적응형 lerp로 보간**(`layers.ts` `robotDisplay`). lerp 시간은 고정 200ms가
  아니라 직전 pose 도착 간격(100~500ms 클램프, `MapRenderer.updateRobotPose`)을 쓰고 heading은 최단각.
  새 pose 도착 시 `from`을 현재 표시중 보간 위치로 전진시켜, 불규칙 도착(WiFi 지터)에도 마커가 옛
  시작점으로 되튀지 않는다. (주행 자취/trail은 제거됨 — 마커는 chevron + 라벨만.)
- **라이다 점 클라우드**: 브릿지가 `/scan`을 TF로 map 프레임 변환 → `scan_points` envelope → 프런트
  `drawScanPoints`가 초록 점(3×3px fillRect 중심정렬, `rgba(34,197,94,0.85)`)으로 렌더(`layers.ts`).
  브릿지는 step=1로 전체 빔(LDS-01 360빔 × 5Hz=1800pts/s)을 그대로 보낸다 — 가시성 우선, 트래픽
  부담 없음. 정적 맵 walls와 정렬되면 위치추정 성공 시각 검증으로도 쓰임. 렌더 순서는 paths 다음·
  robots 이전(마커가 점 위에).
- **배터리 표시 안정화**: `BatteryState.percentage`는 ADC 노이즈로 1~2% 진동해 프런트 `toFixed(0)`
  표시가 끊임없이 바뀌었다. 브릿지 `_on_battery`에서 EMA(α=0.08, 1Hz 발행 기준 시상수 ~12s)로 평활해
  정수 표시가 안정. 첫 샘플은 그대로 채택. 0..1 클램프 후 저장(`topic_subscriber.py`).
- **경로 점선 두 종류**: (a) sky-blue 긴 점선 `[8,6]` = 전역 경로(backend `mission.go` 토폴로지 Dijkstra가
  `path_data(kind=global)`로 emit OR C++ `planned_path` envelope으로 latest-wins 덮어쓰기), (b) 파란 짧은
  점선 `[4,4]` = C++ DWA 동적 회피 호(`local_path` envelope, `${robotId}:local` 키). 빈 points 도착 시
  자연 소거(`drawPaths` length<2 가드). Nav2 `/plan` 구독은 제거됨(주행로직 직접 발행으로 대체).
- pose-estimate는 정규화 좌표(nx/ny/dxN/dyN)로 브로드캐스트 → 클라이언트 캔버스 크기 무관 일치. 화살표 길이 30px 고정.
- 백엔드 WS 클라이언트(`ws_client.go`)는 fast-fail(미연결 시 즉시 에러) + 2s 커맨드 타임아웃.
  연결은 conn-scoped context로 관리 — read 종료 시 write도 함께 정리하고(이전엔 write가 영원히
  블록돼 재연결·health 복구가 막혔음), 끊기면 gateway Cache의 로봇을 비워 유령 로봇을 막는다.

## 도메인·데이터 흐름 (핵심)

- **미션 상태 추적**: 실제 미션 FSM은 ROS2에 있고, 백엔드는 운영자 명령(시작/일시정지/목적지)을
  받아 gateway `Cache`에 status·goal·ETA·route를 추적한다. `mission_state`를 hub로 브로드캐스트하고
  `/state/current`에 머지 → 요약 카드가 채워진다. 미션 시작 시 `missions` 행 Create(예측 ETA),
  초기화/재시작 시 Complete(actual_eta=경과). (`api/mission.go`)
  - **브릿지 라이프사이클과 분리**: `sendCommand`가 브릿지 호출 결과와 무관하게 `updateMissionState`를
    호출한다(2026-06-17 변경). 브릿지 미가동이어도 route·노드 필터·요약 카드·ETA가 동작하고, 응답에
    `bridge_offline: true` 메시지로 운영자에게 봇 정지 상태를 알린다. 봇이 켜지면 자연히 이어진다.
- **MissionStatus**: pending/running/paused/completed/failed/aborted + 물류 FSM 6단계
  `loading`/`formation_driving`/`area_stationing`/`sequential_unloading`/`leader_return`/`mission_queue`.
  **RobotStatus**: idle/moving/picking/error/stopped + TurtleBot 상태 토픽
  `NORMAL`/`SLOW`/`STOP`/`AVOIDING`/`STATIONING`/`UNLOADING`/`RETURN_ALLOWED`/`RETURNING`.
  프론트 `translateMission`/`translateStatus`가 한글 라벨화(로봇 상태 카드 뱃지·요약 카드).
- **ETA**: 시작 노드(리더 최근접 노드, 없으면 `loading`)→goal 토폴로지 최단경로(Dijkstra, edge
  expected_time) × `Estimator.Predict`. route는 SystemState.Route로 노출.
- **맵 노드 필터 + 라이트**: 미션 route가 있으면 맵이 **경로 노드만** 표시. 노드 라이트 —
  주행 대상=빨강, 도달=초록, 지나감/미래=꺼짐(리더 위치 vs 경로, 0.4m 도착 임계로 단조 전진,
  `MapRenderer.computeLights`). 리더 없으면 목적지를 빨강으로.
- **분석 영속화**: 텔레메트리는 `fanout` sink → `BatchWriter.Push` → `telemetry` 테이블. `ChartData`가
  speed(1초 버킷 평균)·mission_time(완료 미션 actual_eta) 집계, 군집 추종 오차는 데이터원 미구현(빈 배열).
  ⚠️ 텔레메트리 timestamp는 epoch millis라 `ws_client`가 wire struct로 받아 `time.UnixMilli`로 변환(직접 언마샬 금지).
- 로봇 상태 카드: `색점 · 한글명 · (x,y)` 좌측, `배터리 · 상태 · 역할(리더/추종) · 연결`은
  `ml-auto`로 우측정렬. 명칭은 `robotLabel`(터틀봇3/팔로워/팔로워1/팔로워2).
  RC카(`rc_car_follower`)는 `battery_state` publisher가 없어 배터리 칸을 조건부로 숨긴다 — 리더·TB3 follower만 배터리 표시.

## 주행로직(C++) 통합 인터페이스

GUI는 별도 PC의 `~/Topology_Graph_test` ROS2 패키지와 amr_interfaces 계약으로 통합 예정. **GUI 측 wiring은 전부 선행 완료** — 브릿지가 `/<ns>/planned_path` `/<ns>/avoidance_path`(둘 다 nav_msgs/Path) 구독 + planned_path/local_path envelope으로 backend→frontend forward. 프런트는 같은 global/local 키에 latest-wins로 적재해 자동 시각화.

**C++ 측이 구현할 것**(서비스 5 + 토픽 3 + main 변경): README §15에 완전한 사양 문서화 — 다른 AI/엔지니어가 그것만 보고 끝까지 구현 가능. 통합 시 그 섹션 따라가면 됨.

backend `mission.go`의 로컬 Dijkstra path_data는 C++ planned_path가 활성화될 때까지 fallback으로 동작. 활성화되면 latest-wins로 C++가 우선됨.

## 알려진 미해결 사항

- 레거시 *Page.tsx 디렉토리 정리(삭제) 미수행.
- 분석 탭 **군집 추종 오차** 차트는 데이터원(팔로워 path-history 추종 편차 산출)이 없어 빈 배열.
- RC카 slot 좌표·로봇팔 작업 좌표·ab_curve_mid 등 일부 토폴로지 노드는 외부에서 추후 수령 예정.

> 해결됨(2026-05-29): `websockets` 미설치로 브릿지가 WS 서빙 못 하던 이슈 →
> `pip install --user 'websockets>=12'`(현재 16.0)로 해결. 브릿지 `:9090` LISTEN +
> 명령 왕복 검증 완료. 새 환경에선 apt(9.1) 말고 pip로 설치할 것.

> 해결됨(2026-06-01): 맵 마커 끊김(amcl_pose 불규칙·점프 + 프런트 lerp snap-back) →
> 마커 pose source를 TF `map→base_footprint`(20Hz, `_on_tf_timer`)로 전환하고, 프런트는
> 위치·heading 모두 도착 간격(100~500ms) 기반 적응형 lerp + 최단각 보간(`robotDisplay`)으로
> 바꿈. 새 pose 도착 시 `from`을 현재 표시중 위치로 전진시켜 되튐 제거. 브릿지 spin 루프는
> 틱당 8콜백 드레인으로 /tf 고빈도에 20Hz 타이머가 굶지 않게 함.

> 해결됨(2026-06-02): 배터리 정수 표시가 1~2% 떨리는 문제 → 브릿지 `_on_battery`에
> EMA(α=0.08, 시상수 ~12s) 적용으로 안정. 동시에 라이다 점 가시성 개선 — 브릿지 scan
> 다운샘플 `step=2→1`로 점 개수 2배, 프런트 `drawScanPoints` 점 크기 2×2→3×3px(중심정렬).

> 해결됨(2026-06-02): 호스트(예: 윈도우) Chrome 접속 시 매번 VM IP를 따로 찾아야
> 했던 불편 → 백엔드 시작 로그에 `open in browser` 라인으로 `local` URL과 비-루프백
> IPv4 인터페이스의 `lan` URL 목록을 함께 출력(`server.go` `lanURLs`). 다른 우분투
> PC에서 띄워도 그 PC의 실시점 IP가 자동 노출.

> 해결됨(2026-06-17): 토폴로지(6/1 새 SLAM)와 GUI `maps/map.pgm`(5/27 옛 SLAM)의
> origin이 (0.58 m, 0.36 m) 어긋나 노드가 벽 바깥으로 빠지던 정렬 이슈 →
> `~/Topology_Graph_test/src/amr_topology/maps/`의 paired 맵(origin `[-1.2, -4.72]`)으로
> 교체. 옛 맵은 `map.pgm.bak_pre_topology_sync_0527` 백업. 토폴로지 재캡처 시 짝지어진
> map.pgm+map.yaml도 같이 가져와야 함.

> 해결됨(2026-06-17): 브릿지 미가동이면 미션 시작이 502로 거부되어 노드 필터·요약
> 카드가 동작 안 하던 문제 → `sendCommand`가 브릿지 결과와 무관하게 `updateMissionState`를
> 호출하도록 변경. 브릿지 실패 시 응답에 `bridge_offline: true` + 한글 알림 메시지.
> 동시에 mock 모드 일괄 제거(mock_client.go·mock_client_test.go·bridge.New의 mock case·
> /health의 mode 필드·MOCK 뱃지). 브릿지 없이 백엔드만 띄워도 미션 컨텍스트는 정상 동작.

> 적용됨(2026-06-25): RC카 팔로워(Topology_Graph_test의 `rc_car_follower_node`)
> 마커를 맵에 실시간 표시 — `bridge/config.yaml`의 `ros.robots`에
> `{id: rc_car_follower, namespace: rc_car}` 활성화. `/rc_car/odom`이
> 이미 `frame_id="map"`(C++ `odom_frame_id` 기본값)으로 encoder dead-reckoning
> pose를 발행하므로 브릿지의 `_on_odom` 폴백 경로가 그대로 동작. 프런트
> `shared.tsx`에 `rc_car_follower → "팔로워"` 라벨 + ROBOT_COLOR(보라) 매핑,
> `RobotStatusCard.tsx`는 RC카에 한해 배터리 칸을 조건부 숨김(C++이 battery_state
> 미발행 → 항상 0% 표시 회피). RC카는 AMCL이 없어 위치추정 보정 없음 — 시간
> 누적 오차 있음, 초기 위치는 C++ 측 `initial_rc_x/y/yaw` 파라미터로만 세팅.

> 적용됨(2026-06-25): 터틀봇3 주행 카메라(Should 단계 stub) 코드 일괄 제거.
> 삭제: `bridge/ros2_bridge/camera_stream.py`, `backend/internal/api/camera.go`,
> `frontend/src/dashboard/camera/`(CameraStreamPage, components.tsx). 정리:
> bridge `config.yaml` camera 섹션, backend `configs/config.yaml` camera 섹션,
> `core/config.go` CameraConfig·필드, `core/router.go` Camera 인터페이스 +
> `/api/v1/camera/status` 라우트, `domain/command.go` CameraStatus struct,
> `cmd/gui_main/main.go` Camera wiring, settings의 CameraSettings(dead code).
> 매니퓰레이터 카메라(`ManipulatorCard.tsx`의 `ManipulatorCameraCard`,
> `/camera/manipulator/stream`)는 유지 — 별도 슬롯, 영향 없음.
