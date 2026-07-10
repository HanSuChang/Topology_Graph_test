# 스마트 물류 관제 시스템

TurtleBot3 군집(Leader + Follower N대)과 매니퓰레이터를 한 화면에서 감시·제어하는 운영 콘솔. 단일 페이지 대시보드 + PWA + 다중 클라이언트(PC ↔ 폰) 실시간 동기화. ROS2 Humble 백엔드 통합.

## 0. 이 사본을 받은 AI CLI / 엔지니어를 위한 통합 가이드

> 이 폴더는 GCS PC의 `~/ros2_ubuntu_ws/gui_system`에서 2026-07-02에 추출한 **소스 전용 사본**이다.
> 통합 목표: 이 GUI(관제 콘솔)를 주행로직 PC의 `~/Topology_Graph_test`(C++ `amr_topology` 패키지)와 잇는 것.
> **GUI 측 wiring은 전부 선행 완료** — 이 사본의 코드는 수정 없이 빌드해 그대로 쓰면 되고,
> 실제 구현 대상은 C++ 주행로직 측이다(사양 전체가 §15에 자족적으로 문서화됨).

### 0.1 이 사본에서 제외된 것 (수신 후 재생성)

| 제외 항목 | 재생성 방법 |
| --- | --- |
| `frontend/node_modules/` | `cd frontend && npm install` |
| `frontend/dist/` (프로덕션 번들) | `cd frontend && npm run build` |
| `backend/bin/gui_main` (Go 바이너리) | `cd backend && go build -buildvcs=false -o ./bin/gui_main ./cmd/gui_main` |
| `backup/sqlite_backup/gui.db` (미션·텔레메트리 DB) | 백엔드 첫 기동 시 `migrations/`로 빈 스키마 자동 생성 |
| `backend/logs/audit.jsonl` (감사 로그) | 첫 운영자 명령 시 자동 생성 |
| `__pycache__/`, `*.tsbuildinfo`, `frontend/dist-node/` | 빌드 시 자동 재생성 |

### 0.2 필수 환경

1. **Go 1.22+** — 빌드에 `-buildvcs=false` 필수(이 폴더는 `.git`이 없어 VCS 스탬프가 실패한다).
2. **Node.js 18+ / npm**
3. **ROS2 Humble**(브릿지·실봇 연동 시) + Python 의존성:
   ```bash
   pip install --user 'websockets>=12' PyYAML
   ```
   ⚠️ apt의 `python3-websockets`(9.1)는 Python 3.10에서 브릿지 코드와 비호환 — 반드시 pip로.
   확인: `python3 -c "import websockets; print(websockets.__version__)"`
4. `ROS_DOMAIN_ID=27` — 봇·PC 모두 일치(`scripts/run_bridge.sh`가 자동 export). `ROS_LOCALHOST_ONLY`는 미설정 또는 0.

### 0.3 수신 직후 빌드 검증 (모두 0 에러가 정상 상태)

```bash
cd backend  && go vet ./... && go build -buildvcs=false -o ./bin/gui_main ./cmd/gui_main
cd ../frontend && npm install && npx tsc -b && npm run build
cd .. && python3 -c "import ast,glob; [ast.parse(open(f).read()) for f in glob.glob('bridge/ros2_bridge/*.py')]"
```

기동: `./scripts/run_backend.sh`(:8080 — 시작 로그에 접속용 LAN URL 출력) + `./scripts/run_bridge.sh`(:9090).
브릿지 없이 백엔드만 띄워도 미션 컨텍스트(route·ETA·노드 필터·요약 카드)는 동작한다(§7.12) — 봇 마커만 부재.
참고: `scripts/run_backend.sh`는 원래 PC 관례로 `~/.local/go/bin`을 PATH에 prepend한다 — Go가 다른 위치에 있어도 PATH에 잡혀 있으면 무해.

### 0.4 통합에서 구현할 것 (전부 C++ 측, 사양은 §15)

| 항목 | 사양 위치 |
| --- | --- |
| `amr_interfaces` 패키지 + `StartMission.srv` 생성 | §15.2 |
| 서비스 서버 6개: `start_mission` + Trigger 5종(pause/resume/stop/emergency_stop/emergency_clear) | §15.3 |
| 상태 토픽 3개: `mission_status`, `planned_path`, `avoidance_path` | §15.4 |
| `mission_loop.cpp` main 변경 — 하드코딩 루프 대신 외부 명령 대기 | §15.6 |
| 단계별 구현 순서(각 단계마다 GUI에서 즉시 확인 가능) | §15.9 |
| C++ 없이 `ros2 topic pub`/`service call`로 wiring 사전 검증 | §15.8 |

### 0.5 함정 목록 (§15.10 요약 + 환경 이슈)

- `mission_status` 문자열은 GUI enum과 **글자까지 일치**해야 한글 라벨이 뜬다(`running`이지 `Running`/`RUNNING` 아님).
- **빈 `nav_msgs/Path`(`poses: []`)는 의미 있는 신호** — GUI 점선 소거에 쓰인다. 미션 종료/회피 종료 시 한 번 발행.
- pause/stop/emergency는 주행 노드 **내부 플래그**로 구현 — 외부에서 `/cmd_vel` 0 Twist를 쏴도 20Hz 주행 루프가 즉시 덮어쓴다.
- route 노드 ID는 `maps/nodes.yaml`(GUI) == `topology.yaml`(C++) **동일 키**(2026-05-29 동기화 완료). 새 노드 추가 시 양쪽 동시 갱신.
- **토폴로지 재캡처 시 짝지어진 map.pgm+map.yaml도 함께 교체** — 현재 `maps/`의 맵은 topology와 짝인 origin `[-1.2, -4.72]`. 맵만 또는 토폴로지만 갈면 노드가 벽 밖으로 어긋난다(2026-06-17에 실제 겪은 이슈).
- TF `map→base_footprint`(AMCL bringup)가 활성이어야 GUI 마커가 부드럽게 갱신된다 — 마커 주력 pose 소스.
- 로봇이 GUI에 안 보이면: 브릿지 `:9090` 기동 → websockets pip 버전 → 같은 서브넷 → `ROS_DOMAIN_ID=27` → `ROS_LOCALHOST_ONLY` → AP 격리 순으로 점검(§13, CLAUDE.md 진단 체인).

### 0.6 코드 규약 / 추가 문서

- 주석은 전부 **한국어**(WebSocket, AMCL, Nav2 등 기술 용어는 영어 유지).
- 프런트 신규 작업은 `frontend/src/dashboard/cards/`에서 — `dashboard/` 아래 옛 `*Page.tsx` 디렉토리들은 라우팅되지 않는 레거시(§7.11).
- `CLAUDE.md`가 프로젝트 전체 지침(AI CLI라면 자동 로드) — 단, 그 안의 GCS IP(192.168.0.7)·Go 설치 경로 등 환경 값은 원래 PC 기준이니 이 PC 값으로 읽어 바꿀 것.

## 1. 시스템 구성

세 개의 런타임이 단일 모노레포(`gui_system/`)에 모여 있다.

| 런타임 | 디렉토리 | 역할 |
| --- | --- | --- |
| **Backend** | `backend/` | Go 1.22 + Gin. Core Server, WebSocket Gateway/Hub, Bridge Client, SQLite/ETA, 감사 로그. 단일 바이너리(`gui_main`). |
| **Frontend** | `frontend/` | React 18 + Vite + TypeScript + Tailwind + Recharts + Framer Motion. PWA(설치형). 단일 화면(`MainDashboardPage`)으로 통합. |
| **Bridge** | `bridge/` | Python `rclpy` + `asyncio`. ROS2 토픽 ↔ 백엔드 WebSocket 매개자. 미가동 시 봇 마커만 부재, 미션 컨텍스트는 백엔드가 자체 추적. |

```
[ROS2: tf(map→base_footprint) / amcl_pose / odom / scan / battery_state / (planned_path, avoidance_path: C++ §15)]
                 │
                 ▼
        bridge (Python, ROS_DOMAIN_ID=27)
                 │ JSON-over-WS
                 ▼
        backend (Go, gui_main, :8080) ─┬─► SQLite (telemetry + missions)
                 │                     └─► JSONL audit log
                 │ WS /ws (hub fan-out)
                 ▼
    ┌──────────────┬───────────────┐
    │ PC 브라우저  │ 폰 PWA(설치)  │   (client_log / pose_estimate 양방향)
    └──────────────┴───────────────┘
```

## 2. 디렉토리 구조

```
gui_system/
├── backend/                     # Go 모듈 — gui_main 단일 바이너리
│   ├── cmd/gui_main/main.go     # 조립자 — 의존성 와이어링만
│   ├── configs/config.yaml      # 서버·브릿지·DB·ETA·인증·맵
│   ├── migrations/              # 001_init_schema.sql …
│   ├── logs/audit.jsonl         # JSONL 감사 로그 (gitignored)
│   └── internal/
│       ├── core/                # Gin, 라우터, 인증, 미들웨어, 정적, 헬스
│       ├── audit/               # JSONL 감사 로거 (file-based)
│       ├── bridge/              # BridgeClient 인터페이스 + ws / grpc
│       │   └── ws_client.go     # 실제 ROS2 브릿지와의 WS 클라이언트 (재연결·envelope dispatch·request_id 매칭)
│       ├── gateway/             # WS Hub, latest-wins 채널, 캐시, idempotency
│       │   ├── envelope.go      # type 상수: robot_pose / mission_state / alert / path_data / scan_points / local_path / planned_path / client_log / pose_estimate ...
│       │   ├── client.go        # per-client 채널 + pushLatest / pushCritical
│       │   └── hub.go           # BroadcastLatest / BroadcastCritical / BroadcastEnvelope
│       ├── api/                 # mission/status/logs/settings 핸들러
│       │   ├── navigation_map/  # /state/map, /map/info, /map/image, slam.go
│       │   └── data_analytics/  # ETA 추정, 미션 이력, 차트, 백업 루프
│       ├── database/            # SQLite + 리포지토리 + 마이그레이션 + 배치 라이터
│       ├── domain/              # core.go(robot+state+mission+task), topology.go(node+edge), command.go(command+system_state+events)
│       └── topology/            # maps/nodes.yaml 로더
├── bridge/                      # Python ROS2 Bridge
│   ├── ros2_bridge/
│   │   ├── __main__.py          # asyncio loop + 트랜스포트 dispatch(ws/grpc) + rclpy spin
│   │   ├── config.py            # 2단계 config 로더 (Go yaml + bridge-local yaml)
│   │   ├── ros_node.py          # BridgeNode 래퍼 + HAS_RCLPY 플래그 (rclpy 유무로 분기)
│   │   ├── topic_subscriber.py  # /amcl_pose, /odom, /scan, /plan, /battery_state 구독 + odom→동작분류(status)
│   │   ├── command_handler.py   # start/pause/resume/emergency/set_pose_estimate 발행
│   │   ├── websocket_server.py  # 백엔드와의 WS + 명령 echo + (rclpy 미존재 시) 5Hz synth producer
│   │   ├── domain_mapper.py     # ROS 메시지 → 도메인 dict 변환 (json_adapter.go와 동일 스키마)
│   │   ├── topic_filter.py      # RateGate 정의 (현재 파이프라인 미연결 — 실제 throttle 없음)
│   │   ├── health_check.py      # 브릿지 readiness
│   │   └── grpc_server.py       # gRPC 트랜스포트 stub
│   ├── config.yaml              # ROS_DOMAIN_ID + 멀티 로봇 namespace
│   └── requirements.txt
├── frontend/
│   ├── src/
│   │   ├── App.tsx, main.tsx
│   │   ├── dashboard/
│   │   │   ├── MainDashboardPage.tsx       # 레이아웃 + 레일 + WS 로그 브리지만 (≈100줄)
│   │   │   ├── cards/                      # 카드별 분리
│   │   │   │   ├── MissionControlCard.tsx
│   │   │   │   ├── PoseEstimateCard.tsx
│   │   │   │   ├── SummaryCard.tsx
│   │   │   │   ├── RobotStatusCard.tsx
│   │   │   │   ├── ManipulatorCard.tsx     # ManipulatorCard + ManipulatorCameraCard
│   │   │   │   ├── LogsStrip.tsx
│   │   │   │   └── shared.tsx              # Btn, Stat, translateMission, ROBOT_COLOR …
│   │   │   ├── layout/                     # TopStatusBar, Card, NavTabs, DashboardLayout
│   │   │   ├── mission_control/
│   │   │   │   ├── MissionCommandDialog.tsx# 아이디·비번 로그인 (React Portal)
│   │   │   │   └── hooks.ts                # useCommandGuard
│   │   │   ├── navigation_map/
│   │   │   │   ├── MapRenderer.ts          # 캔버스 클래스 (마우스 + 터치)
│   │   │   │   ├── MapPanel.tsx            # 맵 패널 + WS 배선 + 위치추정
│   │   │   │   ├── MapCanvas.tsx           # touch-action:none + 합성 레이어
│   │   │   │   ├── layers.ts               # topology/path/robot/costmap/localization 통합
│   │   │   │   ├── mapTypes.ts, coordinateTransform.ts
│   │   │   └── data_analytics/             # AnalyticsPage + charts.tsx (5 차트 통합) + AnalyticsTypes
│   │   ├── hooks/                          # useWebSocket, useMission, useRobotState …
│   │   ├── lib/
│   │   │   ├── api.ts                      # REST 클라이언트
│   │   │   ├── websocket.ts                # 단일 WS 클라이언트 + 자동 재연결
│   │   │   ├── eventLog.ts                 # 통합 로그 버스 + CLIENT_ID 기반 다중 클라이언트 브로드캐스트
│   │   │   └── poseEstimate.ts             # 위치 추정 토글 pub/sub
│   │   └── types/                          # domain/, transport/, api/
│   ├── public/icons/                       # PWA 아이콘 (192/512/maskable/apple-touch/favicon)
│   ├── vite.config.ts                      # VitePWA + /api,/ws 프록시 (5173 → 8080)
│   ├── index.html                          # Apple PWA meta 포함
│   └── package.json
├── maps/                        # SLAM 산출물 + 노드 토폴로지
│   ├── map.pgm                  # SLAM occupancy grid (ROS map_saver)
│   ├── map.yaml                 # resolution, origin, image:
│   └── nodes.yaml               # 노드/엣지 토폴로지 (사전 등록)
├── backup/sqlite_backup/        # SQLite VACUUM INTO 스냅샷 (gitignored)
├── scripts/                     # run_backend.sh, run_bridge.sh (ROS_DOMAIN_ID=27 자동), build_frontend.sh
└── launch/                      # ROS2 launch — gui.launch.py(domain_id 인자 주입), bringup.launch.py
```

## 3. 빠른 시작 (UI/미션 컨텍스트만)

```bash
# 1) 백엔드 빌드 + 가동
cd backend
go build -buildvcs=false -o ./bin/gui_main ./cmd/gui_main
./bin/gui_main --config ./configs/config.yaml &

# 2) 프론트엔드 (개발 서버 — 5173)
cd ../frontend
npm install
npm run dev

# OR 프로덕션(PWA 설치 가능 — 8080에서 백엔드가 dist 서빙)
npm run build
# 8080으로 접속하면 PWA + 정적 서빙
```

브릿지(`:9090`)가 안 떠 있어도 **백엔드의 미션 컨텍스트(route·goal·ETA)는 추적·broadcast** 되므로 미션 시작 시 맵의 노드 필터·요약 카드·노드 라이트가 즉시 동작한다(README §7.12). 봇 라이브 데이터(pose/scan/battery)만 부재 — 봇 마커가 안 보일 뿐 UI 로직은 검증 가능.

## 4. 실제 ROS2 환경 (TurtleBot3, `ROS_DOMAIN_ID=27`)

```bash
# 0) 의존성 (1회) — websockets는 apt(9.1)가 Python 3.10과 안 맞으므로 pip로 설치
pip install --user 'websockets>=12' PyYAML
# (PyYAML은 ROS2 데스크탑 설치에 보통 이미 포함. apt python3-websockets는 피할 것 —
#  9.1 버전이 Python 3.10에서 브릿지가 쓰는 websockets API와 호환되지 않는다.)
# 설치 확인: python3 -c "import websockets; print(websockets.__version__)"

# 1) ROS2 + 도메인 (자동 설정되지만 명시 export도 가능)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=27

# 2) TurtleBot3에 SSH해 봇 띄우기 (도메인 일치 필수)
#    ssh song@<tb3_ip>  → bringup launch

# 3) 워크스테이션에서
cd gui_system
./scripts/build_frontend.sh        # frontend/dist 생성
./scripts/run_backend.sh &         # gui_main (configs/config.yaml의 bridge.type: websocket)
./scripts/run_bridge.sh &          # Python rclpy 브릿지 (ROS_DOMAIN_ID 자동 export)

# 또는 한 줄 launch
ros2 launch ./launch/gui.launch.py domain_id:=27
```

브릿지 가동 → 백엔드가 `ws://localhost:9090` 자동 연결 → `/state/robots`가 실제 로봇 데이터로 채워짐.

**호스트 브라우저(예: 윈도우 Chrome)에서 열기** — 백엔드 시작 시 LAN URL이 로그로 함께 찍힌다:

```
INFO server listening    addr=0.0.0.0:8080
INFO open in browser     local=http://localhost:8080  lan=[http://192.168.0.7:8080]
```

`lan` 옆 URL을 같은 LAN(같은 서브넷, AP 격리 미사용)의 호스트 PC Chrome에 그대로 붙여 접속. 인터페이스가 여럿이면(이더넷+Wi-Fi, docker0 등) 모두 나열되니 LAN 대역만 고른다. VirtualBox 환경이면 어댑터를 **Bridged**로 설정해야 게스트 IP가 LAN에 노출된다. 방화벽: `sudo ufw allow 8080/tcp`. PWA Service Worker는 `localhost`/HTTPS에서만 등록되므로 LAN IP로 접속 시엔 SW 없이(오프라인 캐시 X) 동작 — 기능엔 지장 없음.

## 5. 화면 구성 (PC 데스크탑)

```
┌── TopStatusBar ── 헬스 뱃지 ── 현재 시각 ── 관리자 로그인 ──┐
├──────────────┬───────────────────────────────┬────────────┤
│ 미션 제어    │                               │ 요약       │
│ 위치 추정    │      네비게이션 맵            │ 매니퓰레이터│
│ 로봇 상태    │  (SLAM + 토폴로지 + 마커)     │  카메라    │
│              │                               │ 매니퓰레이터│
│              │                               │  상태      │
├──────────────┴───────────────────────────────┴────────────┤
│         실시간 로그 스트립 (전체 폭)                       │
└────────────────────────────────────────────────────────────┘
```

각 카드:

- **미션 제어** — 목적지 선택, 시작/일시정지/재개/초기화/긴급 정지/긴급 해제. 모든 클릭이 `[ui] → [command]` 로그 + (다중 클라이언트면 다른 세션에도 동기화).
- **위치 추정 (RViz 스타일 2D Pose Estimate)** — 버튼 토글 후 맵에서 마우스(또는 손가락) 누르고 끌기 → 시작점에 초록 마커, 드래그로 화살표 헤딩. 떼면 `/initialpose`로 `PoseWithCovarianceStamped` 발행 + 다른 클라이언트 맵에 파란 마커 4초간 표시.
- **로봇 상태** — Leader/Follower 1줄 칩. 좌측 `색점 · 한글명(터틀봇3/팔로워/팔로워1/2) · (x,y)`, 우측정렬(`ml-auto`)으로 `배터리 · 상태 뱃지 · 역할(리더/추종) · 연결`이 줄 끝에 정렬. 상태 뱃지는 `NORMAL`~`RETURNING`/`이동` 등을 한글로(`translateStatus`). 정렬 순서: leader → follower_1 → follower_2 → rc_car_follower(rank 999). RC카(`rc_car_follower`)는 `battery_state` publisher가 없어 배터리 칸을 조건부 숨김 — 리더·TB3 follower만 배터리 표시.
- **네비게이션 맵** — `MapRenderer` 클래스가 SLAM PGM(전처리됨)을 스트레치 fit으로 패널 4면에 맞붙임. 휠/핀치 줌, 드래그/터치 팬, 노드 탭으로 목적지 선택. **노드·로봇·경로·클릭이 맵과 동일 좌표 변환으로 정렬**(§7.7). 미션 진행 중엔 **경로 노드만 표시 + 노드 라이트**(주행 대상=빨강, 도달=초록, 지나감=꺼짐, §7.13).
- **매니퓰레이터 카메라 / 상태** — MJPEG 스트림 + Pick/Drop 상태 뱃지 + 그리퍼 상태. 두 카드 5:5 분할.
- **요약** — 미션 상태(6단계 한글)·예상 시간(ETA)·목적지(노드 한글명). 운영자 명령에 따라 백엔드가 추적해 `mission_state` WS로 실시간 갱신(§7.12).
- **분석 탭(`/분석`)** — 상단 헤더의 "분석"으로 이동. ETA 예측(예측 vs 실제)·최근 미션 이력·속도·미션 소요 시간·군집 추종 오차 카드. 뷰포트를 꽉 채우는 2행 그리드, 미션 이력은 헤더 고정 내부 스크롤(§7.15).
- **실시간 로그** — UI 클릭·WS `mission_state`·`alert`·`system_log`·다른 클라이언트 `client_log` 통합. 가로·세로 스크롤 모두 지원, 신규 항목 자동 스크롤.

## 6. 화면 구성 (스마트폰 PWA)

`< lg(1024px)`에서 자동 단일 컬럼 스택 + 페이지 전체 세로 스크롤. 헤더는 "물류관제"로 축약. 카드 패딩·텍스트 크기 모바일 친화, 버튼 터치 타겟 ≥44px. 맵 영역은 60vh 고정 슬롯.

**홈 화면 추가 절차:**

| OS | 절차 |
| --- | --- |
| Android Chrome | `http://<gcs_ip>:8080` 접속 → 우상단 ⋮ → 홈 화면에 추가 / 설치 |
| iOS Safari | 동일 URL 접속 → 공유 → 홈 화면에 추가 |

설치 후 아이콘 탭 시 standalone 모드로 풀스크린 실행. 오프라인 시 정적 리소스는 SW 캐시.

## 7. 핵심 아키텍처

### 7.1 WebSocket Envelope

모든 WS 메시지는 동일한 envelope:

```json
{ "type": "robot_pose", "robot_id": "tb3_leader", "timestamp": 1234567890123,
  "seq": 123, "request_id": "uuid", "payload": { ... }, "version": 1 }
```

지원 type: `robot_pose`, `mission_state`, `alert`, `status`, `system_log`, `path_data`, `telemetry`, `command_result`, **`client_log`**, **`pose_estimate`**, **`scan_points`**(라이다 점 클라우드 시각화 §7.9), **`local_path`**(C++ DWA 우회 호, §15), **`planned_path`**(C++ 전역 계획 경로, §15). (`status`는 로봇 동작 변화 — `motion_forward`/`reverse`/`turn_left`/`turn_right`/`stopped` — 이벤트로, 백엔드 `ws_client.go`가 `alert`와 함께 status 채널로 라우팅.)

`mission_state` payload는 미션 컨텍스트를 담는다: `{status, current_goal_node, eta_seconds, route}`. 백엔드가 운영자 명령으로 갱신해 hub로 fan-out하며, 프론트(요약 카드·맵 노드 필터)가 이를 구독한다(§7.12).

### 7.2 Latest-Wins 채널 vs Critical 채널

| 채널 | 크기 | 정책 | 사용처 |
| --- | --- | --- | --- |
| **Latest-Wins** | 1 | 가득 차면 이전 메시지 드랍 후 새 메시지 push | `robot_pose`, `telemetry`, `path_data`, `scan_points`, `local_path`, `planned_path`, `costmap` |
| **Critical** | 64 | 가득 차면 클라이언트 해제(reconnect로 reconciliation) | `mission_state`, `alert`, `command_result`, `client_log`, `pose_estimate` |

### 7.3 Idempotency 캐시

POST 명령은 `X-Request-ID` UUID 동반. `gateway/idem.go`가 TTL 2분 캐시 → 같은 ID 재호출 시 이전 결과 반환 (네트워크 재시도 안전).

### 7.4 다중 클라이언트 동기화 (PC ↔ 폰)

`eventLog.ts`가 `CLIENT_ID`(per-tab UUID) 발급. `logInfo/Warn/Err` 호출 시:
1. 로컬 LogsStrip에 즉시 표시
2. `POST /api/v1/client_log {level, source, message, client_id, ts}` (fire-and-forget, `keepalive:true`)
3. 백엔드 `ClientLog` 핸들러가 `hub.BroadcastEnvelope("client_log", body)`로 fan-out
4. 다른 클라이언트가 WS로 수신 → `emitFromRemote()`가 `client_id` 비교 후 자기 것이면 스킵, 아니면 로컬 push

위치 추정도 같은 패턴: `POST /api/v1/pose_estimate` 성공 시 백엔드가 `pose_estimate` envelope을 echo → 모든 맵에 파란 마커 4초.

### 7.5 감사 로그 (JSONL, file-based)

`backend/logs/audit.jsonl`에 append. Mission Start/Stop, Pause/Resume, Reset, Goal, Emergency Stop/Clear, Login(`user=<id>`) 모두 기록. SQLite와 분리되어 디스크 파손 시에도 운영자 행위 보존.

### 7.6 SQLite + ETA + 영속화

- WAL 모드, `VACUUM INTO`로 7일 보관 스냅샷
- **텔레메트리 적재**: `fanout`이 라이브 송출과 함께 `BatchWriter.Push`로 흘려보내 `telemetry` 테이블에 기록(15초 OR 100건마다 flush). ⚠️ 브릿지 timestamp는 epoch millis라 `ws_client`가 wire struct로 받아 `time.UnixMilli`로 변환한다(time.Time에 직접 언마샬하면 실패해 telemetry가 통째로 드롭됨).
- **미션 이력 적재**: 미션 시작 시 `missions` 행 Create(예측 ETA), 초기화/재시작 시 Complete(`actual_eta`=시작 후 경과). `/api/v1/missions`·분석 탭이 읽음(§7.15).
- ETA: 시작 노드→goal 토폴로지 최단경로(Dijkstra) × 노드별 평균 + pick/drop + 군집 보정. 샘플 5건 미만 시 `Edge.expected_time` 폴백.

### 7.7 SLAM 맵 처리 파이프라인

`backend/internal/api/navigation_map/slam.go`:

1. `decodePGMGray` — P5/P2 PGM 파서, 0~255 그레이 버퍼
2. `removeIslandObstacles(minIslandPixels=20)` — 어두운 픽셀 connected components 라벨링, **20px 미만 컴포넌트만 free(254)로 erase** → 메인 벽 + 내부 장애물 보존, 4px 스캐터 노이즈 제거
3. `findDarkBBox` — **island removal 후** bbox 계산 (메인 벽 외곽선의 타이트 bbox)
4. `cropToBBoxWithAlpha` — bbox 영역만 NRGBA로 옮기고 unknown(205±) → 알파 0
5. `rotateCCW90` — `(x,y)→(y,W-1-x)` 매핑, dim swap (가로 모드)

결과 PNG는 `/api/v1/map/image` 서빙(`Cache-Control: no-store`). `/api/v1/map/info`가 처리 이미지 크기 + **변환 파라미터**(`origin_x/y`, `orig_width/height`, `crop_min_x/y`, `crop_w/h`, `resolution`, `fit:true`)를 반환한다.

프론트 `MapRenderer.drawSlam()`이 캔버스 전체로 **스트레치 fit**(벽이 패널 4면에 맞붙음)하고, **`coordinateTransform.toScreen/fromScreen`이 위 파라미터로 world↔이미지 매핑을 복원**한다(world → 원본 픽셀 → crop → CCW90 → stretch). 그래서 노드·로봇·경로·노드탭·pose가 전부 맵 위 올바른 위치에 정렬된다. heading(chevron·pose 화살표)은 `headingToScreenAngle`로 toScreen 투영각을 써 회전·stretch를 반영(raw cos/sin 금지). 맵 미로드 시엔 단순(월드 원점=캔버스 중앙) 변환으로 폴백.

### 7.8 ROS2 브릿지 토픽 매핑

`bridge/ros2_bridge/topic_subscriber.py`:

| ROS2 토픽 | → 백엔드 envelope | 비고 |
| --- | --- | --- |
| TF `map→base_footprint` | `robot_pose` | **마커 pose 주력 소스(20Hz, `_on_tf_timer`).** odom 빈도로 매끄럽고 map 정합이라 마커가 부드럽다. 주행로직 `mission_loop`이 쓰는 바로 그 소스 |
| `/<ns>/amcl_pose` | `robot_pose`(fallback) | TF 가용 시 push 생략(충돌 방지). TF 미가용 시에만 폴백. 발행이 불규칙·저빈도 + 보정 시 점프 |
| `/<ns>/odom` | `robot_pose`(2차 fallback) + `telemetry`(2Hz 스로틀) + `status`(동작 변화 시) | TF·AMCL 둘 다 침묵해 `_last_pose`가 비었을 때만 pose 드라이브. 속도로 전진/후진/좌·우회전/정지 분류(deadband+0.3s debounce) → 변화 시점에만 status emit |
| `/<ns>/scan` | `alert`(min<0.2m일 때만) **+ `scan_points`**(map 프레임 점 클라우드) | TF로 (r,θ)→(x,y) 변환, **step=1**(전체 빔 — LDS-01 360빔 × 5Hz=1800pts/s, WS 트래픽 여유). 프런트가 맵 위 초록 점(**3×3px 중심정렬**)으로 그림(§7.9). 정적 맵 walls와 정렬되면 위치추정 정상 |
| `/<ns>/battery_state` | (state에 병합) | 0~1 정규화 + **EMA 평활**(α=0.08, 시상수 ~12s). `BatteryState.percentage`의 ADC 노이즈로 인한 프런트 정수 표시(1~2%) 떨림을 제거. 첫 샘플은 그대로 채택 |
| `/<ns>/avoidance_path` | `local_path` | **C++ DWA 우회 호**(nav_msgs/Path). 프런트가 파란 짧은 점선 `[4,4]`로 표시. publisher 미가동 시 무동작(§15) |
| `/<ns>/planned_path` | `planned_path` | **C++ 전역 계획 경로**(nav_msgs/Path). 프런트가 sky-blue 긴 점선으로 표시. backend `mission.go` 로컬 Dijkstra의 `path_data`와 같은 frontend 키에 latest-wins → 활성화 시 자동 우위(§15) |

**RC카 팔로워 특수 케이스** (`rc_car_follower` / namespace `rc_car`, 2026-06-25 적용):
- C++ `rc_car_follower_node`가 `/rc_car/odom`을 `frame_id="map"`(node 기본 `odom_frame_id`)로 encoder dead-reckoning pose 발행 → 브릿지의 odom 폴백 경로가 그대로 동작해 GUI 마커 표시
- `/rc_car/` 네임스페이스 아래 amcl_pose/scan/battery_state/planned_path/avoidance_path/TF 모두 부재 → 콜백 호출 안 됨(무영향)
- 제약: AMCL 없음 → 위치추정 보정 없음, 시간 누적 오차. 위치 추정 버튼은 RC카에 무효(C++ 측 `initial_rc_x/y/yaw` 파라미터로만 초기화). battery_state 없어 프런트 `RobotStatusCard`가 RC카 행 배터리 칸을 조건부 숨김(§7.12 인접 — 라벨/UI 처리는 `shared.tsx` `robotLabel`/`ROBOT_COLOR`에 `rc_car_follower → "팔로워"` 매핑)

`command_handler.py`가 envelope→ROS2 publish 매핑:

| 명령 | → ROS2 |
| --- | --- |
| `start_mission` / `change_goal` | Nav2 `navigate_to_pose` action (있을 때만) |
| `pause` / `emergency_stop` | `/<ns>/cmd_vel` zero Twist + goal cancel |
| `set_pose_estimate` | `/<ns>/initialpose` PoseWithCovarianceStamped |

`HAS_RCLPY=False` 또는 Nav2 미설치 환경에서도 import 에러 없이 stub 응답.

### 7.9 맵 시각화 (마커 · 경로)

- **로봇 마커**: 원형이 아닌 **화살촉(chevron)** 으로, heading 방향을 가리켜 정면을 즉시 인식. 라벨은 한글(터틀봇3 / 팔로워1 / 팔로워2).
- **마커 보간**: 위치·heading 모두 **적응형 lerp**(`layers.ts` `robotDisplay`). lerp 시간은 고정 200ms가 아니라 직전 pose 도착 간격(100~500ms 클램프)을 쓰고 heading은 최단각. 새 pose 도착 시 `from`을 현재 표시중 위치로 전진시켜(`MapRenderer.updateRobotPose`) 불규칙 도착(WiFi 지터)에도 마커가 옛 시작점으로 되튀지 않는다.
- **전역 경로 계획(sky-blue 긴 점선 `[8,6]`)**: 두 소스 중 최신이 표시됨 — (a) backend `mission.go`가 토폴로지 Dijkstra route를 `path_data(kind=global)`로 emit, (b) C++ 주행로직이 `planned_path` envelope으로 발행(§15). 같은 frontend 키 `${robotId}:global`에 latest-wins, C++ publisher 활성화 시 자동 우위. (Nav2 `/plan` 구독은 제거됨.)
- **로컬 회피 호(파란 짧은 점선 `[4,4]`)**: C++ DWA가 동적 장애물을 만났을 때 선택한 `committed_arc`를 `local_path` envelope으로 발행 → `${robotId}:local` 키. 회피 종료 시 빈 points로 자연 소거(§15).
- **라이다 점 클라우드(초록 점)**: 브릿지가 `/scan`을 TF로 map 프레임 변환해 `scan_points` envelope으로 5Hz 흘려보냄. **다운샘플 없음(step=1, 전체 빔)** + **3×3px 중심정렬** 렌더로 가시성을 키웠다. 동적 장애물 위치 + 위치추정 검증(점이 벽과 정렬 = AMCL 정상) 동시 가능.

### 7.10 성능 최적화

- **명령 fast-fail**: 브릿지 미연결 시 `WSClient.SendCommand`가 즉시 에러 반환(타임아웃 2초). 버튼 busy-latch가 바로 풀려 클릭 반응성 확보.
- **MapRenderer**: 30fps 캡 + `visibilitychange` 시 렌더 정지(백그라운드/잠금 시 0fps).
- **브릿지 spin 드레인**: 단일 스레드 executor의 `spin_once`는 틱당 ready 콜백 하나만 처리한다. TF listener(/tf 고빈도) + odom/scan + 20Hz TF 타이머가 함께 돌면 적체되므로, `__main__.py`가 틱마다 8콜백 드레인 + 슬립 5ms로 처리량 천장을 높여 20Hz pose가 굶지 않게 한다.
- **CSS**: `.glass` 카드에 `contain: layout paint style`로 페인트 범위 격리, `backdrop-filter`는 데스크탑만(`lg+`) 4~6px, 모바일은 제거. `<canvas>`는 `translateZ(0)`로 별도 합성 레이어.
- **LogsStrip**: DOM 렌더 최근 80줄 제한(메모리는 300줄 유지).

### 7.11 코드 규약

- **주석은 모두 한글**. 백엔드(Go) · 브릿지(Python) · 프론트엔드(TS/TSX) 전 코드의 주석/docstring을 한글로 통일. 기술 용어(WebSocket, envelope, AMCL, Nav2, SLAM 등)는 영문 유지.
- **파일 크기 sweet spot 150~400줄**: domain 8→3 통합, gateway latest_wins→client.go 흡수, 맵 레이어 5→`layers.ts`, 차트 5→`charts.tsx`. 역으로 440줄 `MainDashboardPage`는 `cards/`로 분리.
- 미사용 레거시 페이지(`overview/`, `swarm_status/`, `settings/`, `logs/`, `robot_status/`, `manipulator_status/`, `NavigationMapPage`, `MissionControlPage`)는 초기 멀티 페이지 구조의 잔재로 라우팅되지 않음 — 향후 삭제 권장. (`camera/`는 2026-06-25에 터틀봇3 주행 카메라 제거와 함께 삭제됨 — 매니퓰레이터 카메라는 `cards/ManipulatorCard.tsx`에서 유지.)

### 7.12 미션 상태 추적 + 요약 카드

실제 미션 FSM은 ROS2(주행 스택)에 있고, 백엔드는 **운영자 명령(GUI 의도)** 을 받아 미션 컨텍스트를 추적한다(`api/mission.go` `updateMissionState`):

- 시작/목적지 변경 → `running` + 목적지 + ETA + route를 gateway `Cache`에 기록
- 일시정지/재개/초기화 → 상태 전이(초기화 시 목적지·ETA·route 클리어)
- 변경 시 `mission_state`를 hub로 브로드캐스트하고, `/state/current`는 브릿지 상태(로봇 라이브)에 Cache의 미션 필드를 덮어써 반환
- **브릿지 라이프사이클과 분리(2026-06-17)**: `sendCommand`가 브릿지 호출 결과와 무관하게 `updateMissionState`를 호출한다. 브릿지 미가동이어도 route·노드 필터·요약 카드·ETA가 즉시 동작하고, 응답의 `bridge_offline: true` + 한글 메시지로 봇 정지 상태만 알린다. 봇이 켜지면 자연히 이어진다(idempotency 캐시가 중복 명령 방지)

도메인 enum:
- **MissionStatus**: pending/running/paused/completed/failed/aborted + 물류 FSM 6단계 `loading`/`formation_driving`/`area_stationing`/`sequential_unloading`/`leader_return`/`mission_queue`
- **RobotStatus**: idle/moving/picking/error/stopped + TurtleBot 상태 토픽 `NORMAL`/`SLOW`/`STOP`/`AVOIDING`/`STATIONING`/`UNLOADING`/`RETURN_ALLOWED`/`RETURNING`

프론트 `translateMission`/`translateStatus`가 한글화. 미션 제어 목적지 드롭다운은 하드코딩 대신 `/state/map`의 노드(loading_zone/stationing_slot/charger)를 받아 한글 라벨로 구성.

### 7.13 경로 노드 필터 + 노드 라이트

미션 route(`SystemState.Route`, 시작→goal 최단경로)가 설정되면 맵이 **경로 노드만** 표시하고(나머지 숨김), 리더 진행에 따라 노드 라이트를 켠다(`MapRenderer`):

- **주행 대상**(다음 노드) = 빨강 글로우, **현재 도달** = 초록 글로우, **지나간/미래** = 꺼짐(평소 색)
- 리더 위치가 다음 경로 노드에 **0.4m** 안에 들면 도착으로 단조 전진(`computeLights`). 리더 미가동 시 목적지를 빨강으로 표시
- route는 `mission_state` payload + `/state/current`로 전달, `MapPanel`이 `renderer.setRoute()`로 적재. 노드 선택(`pickNode`)도 보이는 노드로 한정

### 7.14 브릿지 재연결 + 로봇 캐시 클리어

`ws_client.go`는 conn-scoped context로 read/write 루프를 묶는다 — read가 끝나면 `connCancel`+`Close`로 write를 깨워, 보낼 데이터가 없을 때 write가 죽은 소켓을 못 감지해 `<-writeDone`이 영원히 블록되던(재연결·health 복구가 막히던) 문제를 없앴다. 브릿지 연결이 끊기면 gateway `Cache`의 로봇을 비워(`ClearRobots`, `fanout`이 연결 상태를 폴링) 유령 로봇이 `/state/robots`에 남지 않는다. 재연결되면 `robot_pose`로 다시 채워진다.

### 7.15 분석 탭 (`/분석`)

`AnalyticsPage`(뷰포트 높이를 꽉 채우는 2행 그리드). 데이터는 `/api/v1/missions`·`/api/v1/analytics/chart`에서:

| 카드 | 소스 |
| --- | --- |
| ETA 예측(예측 vs 실제) | `missions` 완료 행의 predicted/actual_eta |
| 최근 미션 이력 테이블 | `missions`(최근 50). 헤더 sticky + 내부 스크롤 |
| 속도 차트 | `telemetry` 속도를 **1초 버킷 평균**(여러 로봇 5Hz 샘플을 군집 평균 한 점으로) → 매끄러운 영역 그래프 |
| 미션 소요 시간 | 완료 미션 actual_eta |
| 군집 추종 오차 | ⚠️ 데이터원 미구현 → 빈 차트 |

차트 X축은 시:분:초(`fmtTime`), 속도 Y축은 0부터. `ChartData`(`chart_data.go`)가 집계.

## 8. 설정 (`backend/configs/config.yaml`)

| 키 | 기본값 | 비고 |
| --- | --- | --- |
| `server.host` / `server.port` | `0.0.0.0:8080` | |
| `server.static_dir` | `../../frontend/dist` | configs 디렉토리 기준 상대, 자동 절대 변환 |
| `bridge.type` | `websocket` | `websocket` / `grpc` |
| `bridge.address` | `localhost:9090` | Python 브릿지 WS |
| `db.path` | `../../backup/sqlite_backup/gui.db` | |
| `db.wal` | `true` | |
| `auth.session_ttl_minutes` | `60` | |
| `emergency.allowed_cidrs` | `127.0.0.1/32` 외 | 긴급 정지 IP 화이트리스트 |
| `emergency.rate_limit_per_minute` | `30` | |
| `audit.path` | `../logs/audit.jsonl` | |
| `eta.min_samples_for_average` | `5` | 미만은 expected_time 폴백 |
| `nodes_file` | `../../maps/nodes.yaml` | |
| `map_yaml` | `../../maps/map.yaml` | image: 파일명 stale 시 같은 디렉토리 첫 `.pgm` 자동 폴백 |
| `migrations_dir` | `../migrations` | |

`bridge/config.yaml` 별도 키:
- `ros.domain_id`: `27` (자동 export)
- `ros.robots`: 기본 활성 `tb3_leader`(namespace `""`) + `rc_car_follower`(namespace `rc_car`, `/rc_car/odom` 구독). TB3 follower 추가 시 `follower_1`/`follower_2` 주석 해제

## 9. REST API 일람

| 메서드 | 경로 | 인증 | 설명 |
| --- | --- | --- | --- |
| GET | `/api/v1/health` | – | 서버/브릿지/DB 상태 |
| GET | `/api/v1/state/current` | – | 통합 시스템 스냅샷 (미션 status·goal·eta·route는 백엔드 Cache 기준으로 머지) |
| GET | `/api/v1/state/robots` | – | 로봇 캐시 (브릿지 끊기면 비워짐) |
| GET | `/api/v1/state/map` | – | 노드 + 엣지 + 로봇 메타 |
| GET | `/api/v1/map/info` | – | 처리 SLAM 맵 메타 + 좌표 변환 파라미터(origin·orig dims·crop bbox) |
| GET | `/api/v1/map/image` | – | 처리된 SLAM PNG |
| GET | `/api/v1/missions` | – | 미션 이력(최근 50, predicted/actual_eta) |
| GET | `/api/v1/analytics` | – | KPI: 최근 미션 수 · 평균 ETA |
| GET | `/api/v1/analytics/chart` | – | 시계열: speed(1초 버킷 평균)·mission_time·formation_error |
| POST | `/api/v1/auth/login` | – | `{username, password_hash}` → session_id 쿠키 |
| POST | `/api/v1/missions/start` | 관리자 | |
| POST | `/api/v1/missions/{pause,resume,reset,goal}` | 관리자 | |
| POST | `/api/v1/emergency/stop` | – (allow-list + RL) | |
| POST | `/api/v1/emergency/clear` | 관리자 | |
| POST | `/api/v1/pose_estimate` | – | `{x, y, theta}` → 브릿지 → `/initialpose` |
| POST | `/api/v1/client_log` | – | `{level, source, message, client_id, ts}` → 모든 클라이언트로 fan-out |
| GET | `/ws` | – | 단일 WebSocket 게이트웨이 |

정적 (PWA):
- `GET /` → index.html
- `GET /assets/*`, `/icons/*` — 정적
- `GET /manifest.webmanifest` — `application/manifest+json`
- `GET /sw.js`, `/registerSW.js`, `/workbox-*.js` — Service Worker
- 알 수 없는 경로 중 확장자 없으면 SPA fallback(index.html), 있으면 404

## 10. PWA 빌드 & 모바일

```bash
cd frontend && npm run build       # dist/sw.js + manifest.webmanifest + workbox 생성
```

manifest (`vite.config.ts`):
- `name`: "스마트 물류 관제 시스템", `short_name`: "물류관제"
- `theme_color`: `#3b82f6`, `display: standalone`, `orientation: any`, `lang: ko`
- icons 3장: 192/512/maskable-512

폰 PWA에서 새 빌드 적용 — Service Worker `registerType: "autoUpdate"`로 백그라운드 갱신. 즉시 반영하려면 PWA를 한 번 종료 후 재실행, 또는 폰 브라우저에서 새로고침.

## 11. 모바일 인터랙션 (터치)

`MapRenderer.attachInteractions()`에 마우스와 터치 핸들러 별도 등록:

| 제스처 | 동작 |
| --- | --- |
| 한 손가락 짧은 탭 | 노드 선택 (목적지 지정) |
| 한 손가락 누르고 끌기 (위치추정 OFF) | 맵 패닝 |
| 한 손가락 누르고 끌기 (위치추정 ON) | 위치+방향 캡처, 5cm 미만 드래그는 무시 |
| 두 손가락 핀치 | 줌 (`vp.scale * (curr_d / prev_d)`, 20~400 클램프) |

캔버스 요소에 `touch-action: none` → 브라우저 기본 더블탭 줌/스크롤 차단.

## 12. 개발 워크플로

```bash
# 백엔드 빌드
cd backend && go vet ./... && go build -buildvcs=false -o ./bin/gui_main ./cmd/gui_main

# 프론트 타입 체크 + 프로덕션 빌드
cd frontend && npx tsc -b && npm run build

# 미션 컨텍스트만 확인 (브릿지 없이도)
# 백엔드만 띄우고 미션 시작 → 노드 필터·요약 카드·ETA 동작 검증 가능 (봇 마커는 부재)
```

## 13. 트러블슈팅

| 증상 | 원인/해결 |
| --- | --- |
| `/api/v1/map/info`가 `{available:false}` | `map_yaml` 경로/이름 확인. yaml의 `image:` stale 시 같은 디렉토리의 첫 `.pgm`로 자동 폴백한다. |
| 맵이 갱신 안 됨 | 백엔드는 매 요청마다 PGM 재디코딩 + `Cache-Control: no-store`. 브라우저 새로고침이면 충분. |
| `go build` VCS 에러 | `-buildvcs=false` 플래그 사용. |
| 브릿지가 "websockets package not installed" 출력 후 `:9090` 미서빙 | `pip install --user 'websockets>=12'` (apt `python3-websockets` 9.1은 Python 3.10에서 브릿지 코드와 비호환). 확인: `python3 -c "import websockets; print(websockets.__version__)"`. |
| 로봇 상태에 "로봇 데이터 없음" | `/state/robots`가 비어있다는 뜻. 브릿지 가동 + 봇과 `ROS_DOMAIN_ID` 일치 확인. 봇 없이도 미션 시작·노드 필터·ETA는 백엔드 단독으로 동작한다(§7.12). |
| 미션 시작이 "브릿지 미연결" 메시지를 띄움 | 명령은 수락됨(노드 필터·요약 카드 동작) — 봇만 정지. `./scripts/run_bridge.sh`로 브릿지 띄우면 즉시 봇이 명령에 응답. 응답 body의 `bridge_offline: true`로 구분. |
| 미션 시작 후 맵에 노드 필터가 안 보임 | (1) 관리자 로그인 했는지, (2) 응답이 정말 `accepted: true`인지 네트워크 탭 확인. 백엔드는 브릿지 가동 여부와 무관하게 `mission_state` envelope(`route` 필드 포함)을 broadcast한다 — 미수신이면 WS `/ws` 연결 자체를 의심. |
| 로그인 다이얼로그가 작게 잘려서 보임 | `.glass`의 `backdrop-filter`가 `position:fixed` containing block을 만든다. 이미 `MissionCommandDialog`가 `createPortal(document.body)`로 회피. |
| 폰의 변경이 PC에 안 보임 | 한쪽 클라이언트가 캐시된 구 번들을 쓰는 중. 폰 PWA 종료 후 재실행 또는 PC `Ctrl+Shift+R`. 서버→PC만 검증하려면 `curl -X POST /api/v1/client_log` 후 PC 로그창 확인. |
| 핀치 줌이 페이지를 줌함 | 캔버스의 `touch-none` 클래스가 적용됐는지 확인 (`MapCanvas.tsx`). |
| 빌드 후 백엔드 재기동 불필요 | 정적 파일은 매 요청 디스크에서 읽음. 백엔드 재기동은 Go 코드를 바꿨을 때만. |
| 미션 진행 중 맵에 일부 노드만 보임 | 정상 — 미션 route 노드만 표시한다(§7.13). 초기화하면 전체 노드 복귀. |
| 분석 탭 차트/이력이 비어 있음 | 미션을 시작/완료하면 이력·ETA가, 텔레메트리(실제 봇 `/odom`)가 흐르면 속도 차트가 채워짐. 군집 추종 오차는 데이터원 미구현. |
| 미션 이력을 초기화하고 싶음 | 백엔드 종료 후 `backup/sqlite_backup/gui.db`(+`-wal`/`-shm`) 삭제 → 재기동 시 빈 스키마 재생성. |
| 호스트(윈도우) Chrome에서 안 열림 | 백엔드 시작 로그의 `open in browser` 줄에서 `lan` URL 확인. 윈도우 PC가 같은 서브넷인지(앞 3옥텟 일치), VirtualBox는 Bridged 어댑터인지, 방화벽이 8080을 막진 않는지(`sudo ufw allow 8080/tcp`), AP가 client isolation을 켜지 않았는지 확인. 빠른 검증: 윈도우 PowerShell `Test-NetConnection <VM_IP> -Port 8080`. |
| 배터리 정수 표시가 1~2%씩 떨림 | 해결됨. 브릿지가 `BatteryState.percentage`에 EMA(α=0.08, 시상수 ~12s)를 적용한다(§7.8). 그래도 떨리면 충전/방전 경계라 0.5% 부근에서 정수 반올림 경계를 넘는 경우 — 정상. |
| 라이다 점이 너무 듬성·작음 | 해결됨. 브릿지 step=1(전체 빔), 프런트 3×3px 중심정렬. 더 크게 보고 싶으면 `frontend/src/dashboard/navigation_map/layers.ts`의 `drawScanPoints` 크기 상수만 조정 후 `npm run build`. |

## 14. 라이선스 / 의존성

내부 프로젝트. 외부 의존성:

**백엔드 (Go):** Gin, gorilla/websocket, modernc.org/sqlite (pure Go), google/uuid, gopkg.in/yaml.v3

**프론트엔드 (Node):** React 18, Vite, TypeScript, Tailwind CSS, Recharts, Framer Motion, vite-plugin-pwa, react-dom

**브릿지 (Python):** rclpy (ROS2 Humble), websockets, PyYAML, nav2_msgs (선택), geometry_msgs, sensor_msgs, nav_msgs

---

## 15. 주행로직(C++) 통합 인터페이스 사양 (`Topology_Graph_test` 측 구현 항목)

GUI(이 repo, GCS PC)는 별도 PC의 ROS2 주행 로직 패키지(`amr_topology`, `~/Topology_Graph_test`)와 약속된 ROS2 인터페이스로 통신한다. 본 섹션은 **주행 측에 구현해야 할 모든 사항**을 정의한다 — 다른 AI CLI/엔지니어가 본 섹션만 참고해도 C++ 측을 끝까지 구현할 수 있도록 자족적으로 작성됐다.

GUI 측(브릿지·백엔드·프런트) wiring은 모두 선행 완료돼 있다 — C++ 측이 본 사양대로 publisher/server를 띄우면 추가 GUI 작업 없이 모든 시각화·제어가 자동 동작한다.

### 15.1 분담 요약

| 책임 | GUI 측(완료) | 주행 측(구현 대상) |
| --- | --- | --- |
| 명령 발생 | 운영자 클릭 → 브릿지가 amr_interfaces 서비스 호출 | **서비스 서버** 5개(StartMission + Trigger×4) |
| 미션 실행 | (없음) | 받은 route 따라 주행, 회피, 정지/재개 |
| 상태 회신 | 브릿지가 토픽 구독·envelope 변환 | **토픽 발행** 3개(mission_status, planned_path, avoidance_path) |
| 시각화 | 프런트 자동 렌더 | (없음 — 이미 적용본 도착하는 모든 데이터 표시) |

### 15.2 신규 ROS2 인터페이스 패키지 `amr_interfaces` 생성

별도 패키지로 두면 양쪽이 동일 인터페이스 빌드 의존만 가지면 되어 향후 코디네이터 노드가 `mission_loop`을 대체해도 GUI·브릿지 무변경.

**디렉토리 구조:**
```
amr_interfaces/
├── CMakeLists.txt
├── package.xml
└── srv/
    └── StartMission.srv
```

**`StartMission.srv`** (운영자가 주행을 시작/목적지 변경):
```
# Request
string[] route       # 경유 노드 ID 순서, 예: ["loading","intersection_1","a_entry","a_leader_slot"]
string   robot_id    # "" = 리더(기본). 멀티로봇 시 namespace id (예: "follower_1")
---
# Response
bool     accepted
string   message     # 사람용 디버그 메시지(거부 이유 등)
```

**`CMakeLists.txt`** 핵심:
```cmake
cmake_minimum_required(VERSION 3.8)
project(amr_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartMission.srv"
)
ament_package()
```

`amr_topology/package.xml`에 의존성 추가:
```xml
<depend>amr_interfaces</depend>
<depend>std_srvs</depend>
<depend>std_msgs</depend>
<depend>nav_msgs</depend>
```

### 15.3 명령 입력 — 서비스 5개 (주행 노드가 서버로 구현)

모두 `/<ns>/` 네임스페이스 아래(단일 로봇이면 `ns=""` → 루트 토픽).

| 서비스 명 | 타입 | 동작 |
| --- | --- | --- |
| `/<ns>/start_mission` | `amr_interfaces/srv/StartMission` | `route` 배열을 받아 즉시 주행 시작(기존 미션은 중단). 주행 중 다시 호출되면 새 route로 전환(=목적지 변경) |
| `/<ns>/pause` | `std_srvs/srv/Trigger` | 현재 주행을 일시정지(상태 유지). cmd_vel을 내부적으로 0으로 묶음 |
| `/<ns>/resume` | `std_srvs/srv/Trigger` | 일시정지된 route 재개. pause가 아닌 상태에서 호출 시 무시 |
| `/<ns>/stop` | `std_srvs/srv/Trigger` | 미션 완전 중단 + 내부 상태 초기화. 빈 planned_path 발행 권장(점선 소거용) |
| `/<ns>/emergency_stop` | `std_srvs/srv/Trigger` | 즉시 정지 + 이후 모든 명령 잠금(emergency_clear까지) |
| `/<ns>/emergency_clear` | `std_srvs/srv/Trigger` | 잠금 해제. 다른 명령이 다시 흐르게 함 |

**구현 시 주의 — `_cmd_vel`은 본인이 발행 중**: 외부에서 `/cmd_vel` 0 Twist를 한 번 쏴도 주행 노드의 20Hz cmd_vel 루프가 즉시 덮어쓴다. **pause/stop/emergency는 내부 상태 변수**(예: `bool paused_`, `bool emergency_`)로 cmd_vel 발행 자체를 중단/0으로 묶어야 함.

### 15.4 상태 출력 — 토픽 3개

| 토픽 | 타입 | 발행 시점 | 페이로드 의미 |
| --- | --- | --- | --- |
| `/<ns>/mission_status` | `std_msgs/String` | FSM 상태 변경 시 (이벤트 기반) | 아래 FSM 6단계 문자열 중 하나 |
| `/<ns>/planned_path` | `nav_msgs/Path` | 미션 시작/목적지 변경 시 1회 | `header.frame_id="map"` + `poses[]`의 position.x/y가 경유 노드 좌표 (3~10점 정도). 미션 종료 시 빈 `poses` 발행해 GUI 점선 소거 |
| `/<ns>/avoidance_path` | `nav_msgs/Path` | DWA `committed_arc` 갱신 시 (~1.4s 주기) | `header.frame_id="map"` + DWA가 선택한 우회 호의 forward-simulated 점들(보통 10~20점). 회피 종료 시 빈 `poses` 발행 |

**`mission_status` 허용 값** (GUI `MissionStatus` enum과 글자까지 일치):
- `pending` / `running` / `paused` / `completed` / `failed` / `aborted`
- 물류 FSM 6단계: `loading` / `formation_driving` / `area_stationing` / `sequential_unloading` / `leader_return` / `mission_queue`
- 추가 권장: `AVOIDING` (DWA 회피 모드 진입 시) — GUI가 마커에 시각 신호 줄 수 있도록

**예시 `nav_msgs/Path` 페이로드** (CLI 테스트용):
```bash
ros2 topic pub -1 /planned_path nav_msgs/msg/Path \
  '{header: {frame_id: "map"}, poses: [
     {pose: {position: {x: 2.47, y: -2.63}}},
     {pose: {position: {x: 2.44, y: -0.02}}},
     {pose: {position: {x: 1.20, y: 0.0}}}
  ]}'
```

### 15.5 기존 인터페이스 (이미 양방향 동작 중 — 변경 불필요)

| 토픽 | 방향 | 역할 |
| --- | --- | --- |
| `/<ns>/cmd_vel` | 봇 ← 주행로직 | 모터 명령 (주행로직만 발행) |
| `/<ns>/initialpose` | AMCL ← 브릿지 | GUI "위치 추정" 결과 |
| `/<ns>/scan` | 브릿지 ← 봇 | 라이다 — 회피 + GUI 점 클라우드 시각화 |
| `/<ns>/odom` | 브릿지 ← 봇 | 속도 telemetry |
| `/<ns>/amcl_pose` | 브릿지 ← AMCL | 위치 fallback |
| `/<ns>/battery_state` | 브릿지 ← 봇 | 배터리 % |
| `/tf` `map→base_footprint` | 브릿지 ← AMCL | 마커 pose 주력 소스(20Hz) + 라이다 좌표 변환 |

### 15.6 주행 노드 구조 변경 사항 (`mission_loop.cpp`)

현재 `mission_loop`의 `main()`은 노드 띄우자마자 `node->run()`을 호출해 하드코딩 A/B 루프를 즉시 시작한다. 통합 후엔 외부 명령 대기 모드로 바뀌어야 한다.

**필요 변경**:

1. **`main()` 단순화**: `run()` 직접 호출 제거. `rclpy.spin()` 등가물(`rclcpp::spin(node)`)만 돌려 서비스/토픽 콜백 대기.
   ```cpp
   int main(int argc, char** argv) {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<MissionLoop>();
     rclcpp::spin(node);   // 명령 대기
     rclcpp::shutdown();
     return 0;
   }
   ```

2. **서비스 서버 5개 + 토픽 publisher 3개 추가** (생성자에서):
   ```cpp
   start_srv_ = create_service<amr_interfaces::srv::StartMission>(
     "start_mission",
     std::bind(&MissionLoop::on_start_mission, this, _1, _2));
   pause_srv_ = create_service<std_srvs::srv::Trigger>("pause", ...);
   resume_srv_ = ...;
   stop_srv_ = ...;
   emergency_stop_srv_ = ...;
   emergency_clear_srv_ = ...;

   status_pub_ = create_publisher<std_msgs::msg::String>("mission_status", 10);
   planned_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 10);
   avoidance_pub_ = create_publisher<nav_msgs::msg::Path>("avoidance_path", 10);
   ```

3. **on_start_mission 핸들러**:
   ```cpp
   void on_start_mission(const StartMission::Request::SharedPtr req,
                         StartMission::Response::SharedPtr res) {
     // 기존 미션 진행 중이면 중단, 새 route로 교체
     std::lock_guard<std::mutex> lock(mission_mu_);
     current_route_ = req->route;
     emergency_ = false;
     paused_ = false;
     // planned_path 즉시 발행
     publish_planned_path(current_route_);
     // mission_status: "running"
     publish_status("running");
     // 별도 스레드/타이머가 current_route_을 실제 주행(go_path)
     res->accepted = true;
     res->message = "mission accepted";
   }
   ```

4. **내부 정지 메커니즘**: `paused_`/`emergency_` 플래그를 `go_path` 루프 내부에서 매 틱 확인. 참이면 `cmd_vel.publish(Twist{})` 후 spin만 돌고 진행 안 함. resume이 오면 다시 진행.

5. **planned_path 발행**: `topology.yaml`에서 route 노드 좌표 lookup → `nav_msgs/Path` 생성 → 발행. **route가 비거나 stop 호출 시 빈 `poses{}` 발행** (GUI 점선 소거용).

6. **avoidance_path 발행**: `try_avoid_front_obstacle` 안에서 `committed_arc`를 갱신할 때마다, DWA가 simulate한 점들을 map 프레임으로 변환해 발행. **회피 종료 시 빈 `poses{}` 발행**.

7. **mission_status 발행**: 상태 전이 시점(start/pause/resume/complete/AVOIDING 진입·종료/emergency)마다 String 한 줄 발행.

### 15.7 네임스페이스 규칙

- 단일 로봇(현재): namespace 비움 → 토픽/서비스가 루트(예: `/start_mission`, `/planned_path`)
- 멀티로봇 시: launch에서 `ros2 run amr_topology mission_loop --ros-args -r __ns:=/tb3_leader` → 모든 토픽이 `/tb3_leader/...` 아래로
- 브릿지 config(`bridge/config.yaml`)는 이미 multi-namespace 지원 — `ros.robots`에 항목 추가만 하면 됨

### 15.8 검증 방법 (C++ 미완성 단계에서도 wiring 점검)

각 토픽을 `ros2 topic pub`으로 흉내 내면 GUI까지 도달이 즉시 검증된다. 예:

```bash
# planned_path 시각화 확인
ros2 topic pub -r 1 /planned_path nav_msgs/msg/Path \
  '{header: {frame_id: "map"}, poses: [{pose: {position: {x: 2.0, y: -2.5}}}, {pose: {position: {x: 2.4, y: 0.0}}}]}'

# avoidance_path 시각화 확인
ros2 topic pub -r 1 /avoidance_path nav_msgs/msg/Path \
  '{header: {frame_id: "map"}, poses: [{pose: {position: {x: 1.0, y: 0.5}}}, {pose: {position: {x: 1.1, y: 0.6}}}]}'

# mission_status 확인 (GUI 요약 카드)
ros2 topic pub -r 1 /mission_status std_msgs/msg/String '{data: "running"}'
```

서비스 호출 측 검증(GUI → C++):
```bash
ros2 service call /pause std_srvs/srv/Trigger
ros2 service call /start_mission amr_interfaces/srv/StartMission \
  '{route: ["loading","a_entry","a_leader_slot"], robot_id: ""}'
```

### 15.9 구현 우선순위 (작은 단위로 통합 가능)

권장 순서 (각 단계 끝나면 GUI에서 즉시 동작 확인):

1. **amr_interfaces 패키지 + StartMission.srv** 빌드 — `ros2 interface show amr_interfaces/srv/StartMission`로 확인
2. **planned_path publisher만** 추가 — `topology.yaml` 노드 좌표로 nav_msgs/Path 발행. GUI 점선이 C++ 출처로 전환됨을 확인
3. **mission_status publisher** 추가 — GUI 요약 카드/상태 뱃지가 한글화돼 표시됨을 확인
4. **start_mission 서비스 서버** + main 변경 — GUI "주행 시작" 버튼이 실제 봇을 보냄
5. **pause/resume/stop/emergency_stop/emergency_clear Trigger 서비스** + 내부 플래그
6. **avoidance_path publisher** — DWA committed_arc 갱신 시점에 호 발행. GUI에 파란 점선 표시

각 단계 후 §15.8의 `ros2 topic pub` / `ros2 service call`로 단일 명령 검증 가능.

### 15.10 GUI 측 brittle한 부분 (구현 중 회피하면 좋을 함정)

- **`mission_status` 문자열은 글자까지 일치**해야 GUI 한글 라벨 매핑이 동작 (`running`이지 `Running`/`RUNNING` 아님)
- **빈 `nav_msgs/Path`(`poses: []`)는 의미 있는 신호** — 점선 소거에 활용됨. publisher가 종료 시 단발성으로 빈 메시지를 한 번만 보내면 충분
- **`/cmd_vel` 외부 0 Twist는 무력화됨** — 내부 정지 플래그 필수
- **route의 노드 ID는 `topology.yaml`/`nodes.yaml`(GUI)과 동일 키** — 좌표는 양쪽 동기화 완료(2026-05-29). 새 노드 추가 시 양쪽 동시 갱신
- **TF `map→base_footprint`** 가 활성이어야 GUI 마커가 부드럽게 갱신됨 (AMCL bringup 필수)
