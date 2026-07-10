import { newViewport, toScreen, fromScreen, headingToScreenAngle, type Viewport, type SlamTransform } from "./coordinateTransform";
import {
  drawTopology,
  pickNode,
  drawPaths,
  drawRobots,
  drawSwarmRelation,
  drawCostmap,
  drawScanPoints,
  robotDisplay,
  type Costmap,
} from "./layers";
import {
  type MapEdge,
  type MapNode,
  type MapPath,
  type MapPoint,
  type MapRobot,
  type PathKind,
  PATH_COLORS,
  ROBOT_COLORS,
} from "./mapTypes";

// MapRenderer는 React 상태와 분리된 의도적인 plain 클래스다. 설계 §6-3에
// 따라 렌더러가 최신 pose / 토폴로지 / 경로의 자체 사본을 보유하고, React
// 컴포넌트는 명령형 update*() 호출로 공급만 한다. 이로써 Canvas 재드로우를
// React의 render 경로 밖에 둔다. 레이어 로직은 layers.ts에 있고, 이 파일은
// 데이터와 draw 루프만 소유한다.
// SlamMapInfo는 서버의 crop+rotate 정리 후 SLAM 이미지의 월드 공간 배치를
// 보유한다. 맵은 월드 원점을 중심으로 하고 resolution × 픽셀 dims로 크기가
// 정해져, 단일 viewport 변환 아래 로봇/경로와 함께 pan·zoom된다.
export interface SlamMapInfo {
  width: number;
  height: number;
  resolution: number;
  worldW: number;
  worldH: number;
}

export class MapRenderer {
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private vp: Viewport;
  private nodes: MapNode[] = [];
  private edges: MapEdge[] = [];
  private robots = new Map<string, MapRobot>();
  private paths = new Map<string, MapPath>();
  // scans는 로봇별 최신 라이다 점 클라우드(map 프레임). latest-wins라
  // 한 사이클만 보관한다(브릿지가 다음 사이클을 보내면 교체).
  private scans = new Map<string, MapPoint[]>();
  private costmap: Costmap | null = null;
  private slamImg: HTMLImageElement | null = null;
  private slamInfo: SlamMapInfo | null = null;
  // SLAM 이미지의 zoom/pan 변환 anchor. 이 scale에서 이미지가 캔버스를
  // 정확히 채우며, 이 값 대비 현재 scale이 맵의 시각적 zoom을 구동한다.
  private slamAnchorScale = 0;
  // world↔이미지 매핑 기하(백엔드 /map/info). fitToMap에서 anchorScale을
  // 붙여 vp.slam으로 싣는다. null이면 노드/로봇이 단순(중앙) 변환을 쓴다.
  private slamGeom: Omit<SlamTransform, "anchorScale"> | null = null;
  // 현재 미션 경로(노드 id 순서, start→…→goal). 비어 있으면 전체 토폴로지
  // 노드를 표시하고, 차 있으면 경로 노드만 표시한다.
  private route: string[] = [];
  // 경로 진행 인덱스(단조 증가). route[arrivedIdx]=현재 도달(초록),
  // route[arrivedIdx+1]=주행 대상(빨강). 리더가 다음 노드에 도착하면 전진.
  private arrivedIdx = 0;
  private dragging = false;
  private lastMouse: { x: number; y: number } | null = null;
  private rafId: number | null = null;
  private clickHandler: ((node: MapNode) => void) | null = null;
  // 위치추정 모드(RViz 스타일 2D Pose Estimate). 활성 시 mouse down이
  // 찍은 위치를 월드 좌표로 캡처하고, 드래그가 heading 벡터를 갱신하며,
  // mouseup이 등록된 콜백에 pose를 commit한다. 마커 + 화살표 그리기는
  // render()에서 일어난다.
  private poseMode = false;
  private poseStart: { x: number; y: number } | null = null;
  private poseEnd: { x: number; y: number } | null = null;
  // pose 드래그의 캔버스 픽셀 위치 — 정규화(0..1) 좌표 계산에 쓰여,
  // 각 클라이언트 캔버스 크기와 무관하게 broadcast envelope이 같은 시각
  // 콘텐츠를 기술하게 한다.
  private poseStartCanvas: { cx: number; cy: number } | null = null;
  private poseEndCanvas: { cx: number; cy: number } | null = null;
  private poseCommitCb:
    | ((x: number, y: number, theta: number, nx: number, ny: number, dxN: number, dyN: number) => void)
    | null = null;
  // 터치 상태 — 마우스 드래그/휠을 폰/태블릿 제스처로 미러링한다.
  // `touchPanLast`는 한 손가락 패닝을 위한 마지막 손가락 위치를 추적하고,
  // `pinchPrev`는 핀치 줌을 위한 두 손가락 사이 이전 거리다.
  private touchPanLast: { x: number; y: number } | null = null;
  private pinchPrev: number | null = null;
  // 원격 pose 마커 — 다른 클라이언트가 WS broadcast로 위치추정을 commit할
  // 때 설정된다. 만료 timestamp 후 자동으로 사라져 마커가 영원히 남지 않게
  // 한다.
  private remotePose:
    | { x: number; y: number; theta: number; expires: number }
    | null = null;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    const ctx = canvas.getContext("2d");
    if (!ctx) throw new Error("canvas 2d not supported");
    this.ctx = ctx;
    this.vp = newViewport(canvas.width, canvas.height);
    this.attachInteractions();
    this.start();
  }

  setTopology(nodes: MapNode[], edges: MapEdge[]) {
    this.nodes = nodes;
    this.edges = edges;
  }

  // setRoute는 현재 미션 경로를 설정한다. 경로가 바뀌면 진행 인덱스를
  // 리셋한다(새 미션은 시작 노드부터). 빈 배열이면 전체 노드 표시로 복귀.
  setRoute(route: string[]) {
    const next = route ?? [];
    const changed =
      next.length !== this.route.length || next.some((v, i) => v !== this.route[i]);
    this.route = next;
    if (changed) this.arrivedIdx = 0;
  }

  // visibleNodes/Edges는 경로가 설정돼 있으면 경로 노드만, 아니면 전체를
  // 반환한다. 렌더와 노드 선택(pickNode) 양쪽에서 동일하게 쓴다.
  private visibleNodes(): MapNode[] {
    if (this.route.length === 0) return this.nodes;
    const set = new Set(this.route);
    return this.nodes.filter((n) => set.has(n.id));
  }

  private visibleEdges(): MapEdge[] {
    if (this.route.length === 0) return this.edges;
    const set = new Set(this.route);
    return this.edges.filter((e) => set.has(e.from) && set.has(e.to));
  }

  // computeLights는 리더 위치로 경로 진행을 추정해 노드 라이트를 만든다.
  // 리더가 다음 경로 노드에 ARRIVE(0.4m) 안에 들면 도착으로 단조 전진한다.
  // 현재 도달 노드=초록(arrived), 다음 주행 대상=빨강(target), 나머지(지나간/
  // 미래)=꺼짐. 리더가 없으면 목적지(마지막)만 빨강으로 둔다.
  private computeLights(): Map<string, "target" | "arrived"> {
    const lights = new Map<string, "target" | "arrived">();
    const route = this.route;
    if (route.length === 0) return lights;
    const leader = this.robots.get("tb3_leader") ?? this.robots.values().next().value;
    if (!leader) {
      lights.set(route[route.length - 1], "target");
      return lights;
    }
    const lx = leader.target?.x ?? leader.x;
    const ly = leader.target?.y ?? leader.y;
    const ARRIVE = 0.4;
    while (this.arrivedIdx + 1 < route.length) {
      const nn = this.nodes.find((n) => n.id === route[this.arrivedIdx + 1]);
      if (nn && Math.hypot(nn.x - lx, nn.y - ly) <= ARRIVE) this.arrivedIdx++;
      else break;
    }
    const arrivedId = route[this.arrivedIdx];
    if (arrivedId) lights.set(arrivedId, "arrived");
    const targetId = route[this.arrivedIdx + 1];
    if (targetId) lights.set(targetId, "target");
    return lights;
  }

  onNodeClick(handler: (n: MapNode) => void) {
    this.clickHandler = handler;
  }

  // 위치추정 입력 모드 토글. 끌 때 진행 중이던 pick을 비워, 다음 세션에
  // 오래된 마커가 남지 않게 한다.
  setPoseEstimateMode(active: boolean): void {
    this.poseMode = active;
    if (!active) {
      this.poseStart = null;
      this.poseEnd = null;
    }
  }

  // pose 드래그 후 운영자가 마우스/손가락을 뗄 때 호출된다. World
  // (x, y, theta)는 GUI 프레임 값이고, (nx, ny, dxN, dyN)은 정규화 캔버스
  // 좌표(0..1)와 방향이라, 캔버스 크기 차이(PC vs 폰)와 무관하게 다중
  // 클라이언트 미러가 같은 이미지 콘텐츠에 안착한다.
  onPoseEstimate(
    cb: (x: number, y: number, theta: number, nx: number, ny: number, dxN: number, dyN: number) => void,
  ): void {
    this.poseCommitCb = cb;
  }

  // showRemotePose는 다른 대시보드 클라이언트가 WebSocket으로 보낸 pose
  // 마커를 월드 좌표로 표시한다. 로봇 마커와 같은 toScreen 변환을 쓰므로
  // 캔버스 크기/종횡비/zoom·pan이 다른 클라이언트(PC↔폰)에서도 같은 월드
  // 위치에 마커가 안착한다.
  showRemotePose(x: number, y: number, theta: number, ttlMs = 4000): void {
    this.remotePose = { x, y, theta, expires: performance.now() + ttlMs };
  }

  updateRobotPose(id: string, x: number, y: number, theta: number) {
    const prev = this.robots.get(id);
    const color = ROBOT_COLORS[id] ?? "#94a3b8";
    const now = performance.now();
    if (!prev) {
      this.robots.set(id, { id, x, y, theta, color, fromTheta: theta, lastUpdate: now });
      return;
    }
    // 새 pose 도착 시 from을 "현재 표시중" 보간 위치로 먼저 전진시킨다. 직전
    // lerp가 끝나기 전에 다음 샘플이 오면(불규칙 도착) 마커가 옛 시작점으로
    // 되튀던 현상을 막는다.
    const disp = robotDisplay(prev, now);
    prev.x = disp.x;
    prev.y = disp.y;
    prev.fromTheta = disp.theta;
    // 직전 업데이트와의 실제 간격을 lerp 시간으로 쓴다. TF가 20Hz로 들어오면
    // RViz2처럼 거의 실시간으로 따라가야 하므로 보간은 짧게 유지한다.
    const interval = now - (prev.lastUpdate ?? now);
    prev.lerpDur = Math.min(220, Math.max(60, (interval || 80) * 1.2));
    prev.lastUpdate = now;
    // 새 목표 위치·heading. robotDisplay가 from→target으로 위치와 heading을
    // 함께 보간한다(heading은 최단각).
    prev.target = { x, y };
    prev.targetTheta = theta;
    prev.theta = theta;
    prev.lerpStart = now;
  }

  updatePath(robotId: string, kind: PathKind, points: MapPoint[]) {
    if (kind === "global" && points.length < 2) return;
    this.paths.set(`${robotId}:${kind}`, { kind, points, color: PATH_COLORS[kind] });
  }

  updateScan(robotId: string, points: MapPoint[]) {
    this.scans.set(robotId, points);
  }

  updateCostmap(cm: Costmap | null) {
    this.costmap = cm;
  }

  // SLAM 맵 메타데이터 + 이미지를 비동기로 가져온다. 동일 URL에 대해
  // idempotent라 재연결 재시도가 백엔드를 두드리지 않는다.
  async loadSlamMap(infoUrl = "/api/v1/map/info"): Promise<void> {
    try {
      const resp = await fetch(infoUrl);
      if (!resp.ok) return;
      const info = await resp.json();
      if (!info?.available || !info?.image_url) return;
      const img = new Image();
      img.crossOrigin = "anonymous";
      // 하드 새로고침 없이 map.pgm 교체가 보이도록 cache-bust.
      img.src = `${info.image_url}?t=${Date.now()}`;
      await new Promise<void>((resolve, reject) => {
        img.onload = () => resolve();
        img.onerror = () => reject(new Error("slam image load failed"));
      });
      const resolution = typeof info.resolution === "number" && info.resolution > 0 ? info.resolution : 0.05;
      const worldW = info.width * resolution;
      const worldH = info.height * resolution;
      this.slamImg = img;
      this.slamInfo = {
        width: info.width,
        height: info.height,
        resolution,
        worldW,
        worldH,
      };
      // world↔이미지 매핑 기하. 백엔드가 변환 파라미터를 주면 노드/로봇/
      // 경로/클릭이 맵과 동일 변환을 쓰도록 저장한다. 누락(구버전 백엔드)
      // 시 null로 두면 단순 변환으로 폴백한다.
      if (
        typeof info.origin_x === "number" && typeof info.origin_y === "number" &&
        typeof info.orig_height === "number" &&
        typeof info.crop_w === "number" && typeof info.crop_h === "number"
      ) {
        this.slamGeom = {
          originX: info.origin_x,
          originY: info.origin_y,
          res: resolution,
          origH: info.orig_height,
          minX: typeof info.crop_min_x === "number" ? info.crop_min_x : 0,
          minY: typeof info.crop_min_y === "number" ? info.crop_min_y : 0,
          cw: info.crop_w,
          ch: info.crop_h,
        };
      } else {
        this.slamGeom = null;
      }
      // Auto-fit: 전체 맵이 현재 캔버스 안에 들어가는 viewport scale을
      // 고른다. 이후 pan/zoom은 맵을 나머지와 함께 움직여, 네비게이션
      // 패널에서 스크롤하면 모든 레이어가 함께 확대된다는 운영자의 기대에
      // 맞춘다.
      this.fitToMap();
    } catch {
      // 맵은 선택 사항 — 로드 실패 시 그라데이션 배경만 보인다. UI에는
      // 따로 표시하지 않는다.
    }
  }

  destroy() {
    if (this.rafId != null) cancelAnimationFrame(this.rafId);
  }

  // 캔버스 render 루프를 ~30fps로 제한하고 탭/PWA가 숨겨지면 렌더를
  // 건너뛴다. 맵은 200ms lerp 애니메이션에만 연속 재드로우가 필요하고,
  // 나머지는 충분히 정적이라 30fps가 60fps와 구분되지 않으면서 CPU는
  // 절반을 쓴다. 백그라운드 스로틀링은 메인 스레드를 더 풀어줘 명령 버튼
  // 클릭이 더 빠릿하게 느껴지게 한다.
  private start() {
    const targetFps = 30;
    const minDelta = 1000 / targetFps;
    let last = 0;
    const loop = (ts: number) => {
      if (document.hidden) {
        // 긴 sleep 예약 — 페이지가 다시 보이면 visibilitychange 리스너가
        // rAF 사이클을 재개한다.
        this.rafId = null;
        return;
      }
      if (ts - last >= minDelta) {
        last = ts;
        this.render();
      }
      this.rafId = requestAnimationFrame(loop);
    };
    this.rafId = requestAnimationFrame(loop);
    // 사용자가 탭으로 돌아오거나 폰 잠금을 해제하면 재개한다.
    document.addEventListener("visibilitychange", () => {
      if (!document.hidden && this.rafId == null) {
        last = 0;
        this.rafId = requestAnimationFrame(loop);
      }
    });
  }

  private render() {
    const { ctx, vp, canvas } = this;
    vp.width = canvas.width;
    vp.height = canvas.height;

    // 라이트 글래스 맵 배경: 옅은 cyan/violet 워시로 캔버스가 주변 glass
    // 패널과 어우러지게 한다.
    const grad = ctx.createLinearGradient(0, 0, canvas.width, canvas.height);
    grad.addColorStop(0, "rgba(224,242,254,0.55)");
    grad.addColorStop(1, "rgba(237,233,254,0.55)");
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // 그리드 라인 — SLAM 맵이 로드되지 않았을 때만. 맵 벽이 1미터 그리드
    // 보다 운영자에게 더 강한 공간 기준을 주기 때문이다.
    if (!this.slamImg) {
      ctx.strokeStyle = "rgba(148,163,184,0.30)";
      ctx.lineWidth = 1;
      for (let m = -10; m <= 10; m++) {
        const p1 = toScreen(vp, { x: m, y: -10 });
        const p2 = toScreen(vp, { x: m, y: 10 });
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
        const q1 = toScreen(vp, { x: -10, y: m });
        const q2 = toScreen(vp, { x: 10, y: m });
        ctx.beginPath();
        ctx.moveTo(q1.x, q1.y);
        ctx.lineTo(q2.x, q2.y);
        ctx.stroke();
      }
    }

    this.drawSlam();
    drawCostmap(ctx, vp, this.costmap);
    drawTopology(ctx, vp, this.visibleNodes(), this.visibleEdges(), this.computeLights());
    drawPaths(ctx, vp, this.paths);
    drawScanPoints(ctx, vp, this.scans);
    drawRobots(ctx, vp, this.robots, performance.now());
    drawSwarmRelation(ctx, vp, this.robots);
    this.drawPoseEstimate();
  }

  // drawPoseEstimate는 진행 중 / 확정된 위치추정 오버레이를 렌더한다:
  // 찍은 위치의 초록 원 + 운영자가 드래그하는 heading 방향을 가리키는
  // 화살표. 또한 연결된 다른 대시보드 클라이언트(PC ↔ 폰 미러)가 확정한
  // pose에 대해 반투명 파란 마커를 렌더한다.
  private drawPoseEstimate() {
    const { ctx, vp } = this;
    // 원격 pose 마커를 먼저(운영자가 동시에 찍는 중이면 자기 진행 중
    // 오버레이가 위에 그려지도록). TTL 후 만료된다.
    const rp = this.remotePose;
    if (rp) {
      if (performance.now() > rp.expires) {
        this.remotePose = null;
      } else {
        // 월드 좌표(toScreen)로 렌더한다 — 로봇 마커·로컬 pose 마커와 동일한
        // 변환이라, 캔버스 크기/종횡비/zoom·pan이 다른 PC와 폰에서도 같은
        // 월드(=맵) 위치에 마커가 안착한다. (정규화 캔버스 좌표는 두 기기의
        // 뷰가 동일해야만 일치하므로 위치가 어긋났다.)
        const tl = toScreen(vp, { x: rp.x, y: rp.y });
        ctx.save();
        ctx.fillStyle = "rgba(59,130,246,0.85)";
        ctx.strokeStyle = "rgba(37,99,235,1)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(tl.x, tl.y, 7, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        // 화살표 길이는 30px 고정. 방향은 world theta를 toScreen으로 투영해
        // 얻은 화면 각도를 쓴다 — 맵이 CCW90 회전·stretch돼도 위치추정 드래그가
        // 화면에서 보였던 방향과 마커가 일치한다. (단순 cos/sin은 맵 변환을
        // 무시해 방향이 틀어졌다.)
        const arrowLen = 30;
        const screenAng = headingToScreenAngle(vp, { x: rp.x, y: rp.y }, rp.theta);
        const ex = tl.x + Math.cos(screenAng) * arrowLen;
        const ey = tl.y + Math.sin(screenAng) * arrowLen;
        ctx.strokeStyle = "rgba(37,99,235,1)";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(tl.x, tl.y);
        ctx.lineTo(ex, ey);
        ctx.stroke();
        // 화살촉.
        const head = 10;
        ctx.fillStyle = "rgba(37,99,235,1)";
        ctx.beginPath();
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex - head * Math.cos(screenAng - Math.PI / 6), ey - head * Math.sin(screenAng - Math.PI / 6));
        ctx.lineTo(ex - head * Math.cos(screenAng + Math.PI / 6), ey - head * Math.sin(screenAng + Math.PI / 6));
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }
    }
    const s = this.poseStart;
    const e = this.poseEnd;
    if (!s) return;
    const pStart = toScreen(vp, s);
    ctx.save();
    // 위치 마커.
    ctx.fillStyle = "rgba(16,185,129,0.9)";
    ctx.strokeStyle = "rgba(5,150,105,1)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(pStart.x, pStart.y, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    // Heading 화살표.
    if (e) {
      const pEnd = toScreen(vp, e);
      const dx = pEnd.x - pStart.x;
      const dy = pEnd.y - pStart.y;
      if (Math.hypot(dx, dy) > 4) {
        ctx.strokeStyle = "rgba(5,150,105,1)";
        ctx.fillStyle = "rgba(5,150,105,1)";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(pStart.x, pStart.y);
        ctx.lineTo(pEnd.x, pEnd.y);
        ctx.stroke();
        // 화살촉.
        const ang = Math.atan2(dy, dx);
        const head = 10;
        ctx.beginPath();
        ctx.moveTo(pEnd.x, pEnd.y);
        ctx.lineTo(pEnd.x - head * Math.cos(ang - Math.PI / 6), pEnd.y - head * Math.sin(ang - Math.PI / 6));
        ctx.lineTo(pEnd.x - head * Math.cos(ang + Math.PI / 6), pEnd.y - head * Math.sin(ang + Math.PI / 6));
        ctx.closePath();
        ctx.fill();
      }
    }
    ctx.restore();
  }

  // drawSlam은 anchor scale에서 SLAM 이미지를 캔버스에 stretch-fit하고
  // (k=1 → 벽이 4면에 닿고 여백 0), 이후 viewport에 선형으로 scale해
  // wheel/drag 시에도 맵이 로봇과 함께 zoom·pan되게 한다. 이미지는 서버
  // 측에서 벽 외곽선에 타이트하게 크롭됐다; 종횡비는 의도적으로 보존하지
  // 않는다 — 운영자 피드백상 "여백 없음"이 "왜곡 없음"보다 우선이다.
  private drawSlam() {
    const { ctx, canvas, vp } = this;
    const img = this.slamImg;
    if (!img || this.slamAnchorScale <= 0) return;
    const k = vp.scale / this.slamAnchorScale;
    const w = canvas.width * k;
    const h = canvas.height * k;
    const x = canvas.width / 2 * (1 - k) + vp.originX;
    const y = canvas.height / 2 * (1 - k) + vp.originY;
    ctx.save();
    ctx.globalAlpha = 0.95;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(img, x, y, w, h);
    ctx.restore();
  }

  // fitToMap은 현재 viewport scale을 SLAM anchor로 기록한다. 이 scale
  // 에서 stretch-fit 맵이 모든 가장자리에 벽을 닿게 캔버스를 정확히 채우며,
  // 여기서의 모든 zoom은 맵을 비례적으로 확대/축소해 wheel + drag가 계속
  // 동작하게 한다.
  fitToMap() {
    const { width, height } = this.canvas;
    if (width <= 0 || height <= 0) return;
    this.vp.originX = 0;
    this.vp.originY = 0;
    this.slamAnchorScale = this.vp.scale;
    // 노드/로봇/경로/클릭이 맵과 같은 변환을 쓰도록 viewport에 slam 변환을
    // 싣는다. anchorScale은 drawSlam이 쓰는 slamAnchorScale과 동일해야 한다.
    this.vp.slam = this.slamGeom
      ? { anchorScale: this.vp.scale, ...this.slamGeom }
      : undefined;
  }

  private attachInteractions() {
    this.canvas.addEventListener(
      "wheel",
      (e) => {
        e.preventDefault();
        const factor = e.deltaY > 0 ? 0.9 : 1.1;
        this.vp.scale = Math.max(20, Math.min(400, this.vp.scale * factor));
      },
      { passive: false },
    );

    // 아래 위치추정 로직을 위해 window-event 마우스 위치를 canvas-local +
    // world 좌표로 변환한다.
    const canvasPoint = (e: MouseEvent) => {
      const rect = this.canvas.getBoundingClientRect();
      return { cx: e.clientX - rect.left, cy: e.clientY - rect.top };
    };

    this.canvas.addEventListener("mousedown", (e) => {
      if (this.poseMode) {
        const { cx, cy } = canvasPoint(e);
        this.poseStart = fromScreen(this.vp, cx, cy);
        this.poseEnd = this.poseStart;
        this.poseStartCanvas = { cx, cy };
        this.poseEndCanvas = { cx, cy };
        return;
      }
      this.dragging = true;
      this.lastMouse = { x: e.clientX, y: e.clientY };
    });
    window.addEventListener("mousemove", (e) => {
      if (this.poseMode && this.poseStart) {
        const { cx, cy } = canvasPoint(e);
        this.poseEnd = fromScreen(this.vp, cx, cy);
        this.poseEndCanvas = { cx, cy };
        return;
      }
      if (!this.dragging || !this.lastMouse) return;
      this.vp.originX += e.clientX - this.lastMouse.x;
      this.vp.originY += e.clientY - this.lastMouse.y;
      this.lastMouse = { x: e.clientX, y: e.clientY };
    });
    window.addEventListener("mouseup", () => {
      this.commitPoseIfReady();
      this.dragging = false;
      this.lastMouse = null;
    });

    this.canvas.addEventListener("click", (e) => {
      if (this.poseMode) return;
      if (!this.clickHandler) return;
      const rect = this.canvas.getBoundingClientRect();
      const node = pickNode(this.vp, this.visibleNodes(), e.clientX - rect.left, e.clientY - rect.top);
      if (node) this.clickHandler(node);
    });

    // ── 터치 핸들러 — 위 마우스 드래그·휠 줌·클릭 흐름의 폰/태블릿
    //    대응. preventDefault는 운영자가 캔버스와 상호작용하는 동안
    //    브라우저의 페이지 스크롤·줌을 막는다(MapCanvas.tsx 캔버스
    //    요소의 `touch-action: none`과 맞물림).

    const touchPoint = (t: Touch) => {
      const rect = this.canvas.getBoundingClientRect();
      return { cx: t.clientX - rect.left, cy: t.clientY - rect.top };
    };
    const pinchDistance = (a: Touch, b: Touch) =>
      Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY);

    this.canvas.addEventListener(
      "touchstart",
      (e) => {
        e.preventDefault();
        if (e.touches.length === 2) {
          // 두 손가락 핀치 시작.
          this.pinchPrev = pinchDistance(e.touches[0], e.touches[1]);
          this.touchPanLast = null;
          this.poseStart = null;
          this.poseEnd = null;
          this.poseStartCanvas = null;
          this.poseEndCanvas = null;
          return;
        }
        const t = e.touches[0];
        if (this.poseMode) {
          const { cx, cy } = touchPoint(t);
          this.poseStart = fromScreen(this.vp, cx, cy);
          this.poseEnd = this.poseStart;
          this.poseStartCanvas = { cx, cy };
          this.poseEndCanvas = { cx, cy };
          this.touchPanLast = null;
          return;
        }
        this.touchPanLast = { x: t.clientX, y: t.clientY };
      },
      { passive: false },
    );

    this.canvas.addEventListener(
      "touchmove",
      (e) => {
        e.preventDefault();
        if (e.touches.length === 2 && this.pinchPrev != null) {
          const d = pinchDistance(e.touches[0], e.touches[1]);
          if (d > 0) {
            const ratio = d / this.pinchPrev;
            this.vp.scale = Math.max(20, Math.min(400, this.vp.scale * ratio));
            this.pinchPrev = d;
          }
          return;
        }
        const t = e.touches[0];
        if (this.poseMode && this.poseStart) {
          const { cx, cy } = touchPoint(t);
          this.poseEnd = fromScreen(this.vp, cx, cy);
          this.poseEndCanvas = { cx, cy };
          return;
        }
        if (this.touchPanLast) {
          this.vp.originX += t.clientX - this.touchPanLast.x;
          this.vp.originY += t.clientY - this.touchPanLast.y;
          this.touchPanLast = { x: t.clientX, y: t.clientY };
        }
      },
      { passive: false },
    );

    this.canvas.addEventListener(
      "touchend",
      (e) => {
        e.preventDefault();
        // pose 모드에서 마지막 손가락이 떨어질 때 pose commit.
        if (e.touches.length === 0) {
          this.commitPoseIfReady();
        }
        // 핀치가 끝났으면(두 손가락 제스처에서 한 손가락이 떨어짐) 핀치
        // 기준선을 비운다; 남은 손가락이 이 end 이벤트로부터 새 pan을
        // 시작하면 안 된다.
        if (e.touches.length < 2) {
          this.pinchPrev = null;
        }
        if (e.touches.length === 0) {
          this.touchPanLast = null;
        }
        // 짧은 탭(드래그 없음)에서 노드 선택 — pose 모드가 아니고 멀티
        // 핑거 제스처 직후가 아닐 때만.
        if (
          e.touches.length === 0 &&
          !this.poseMode &&
          this.clickHandler &&
          e.changedTouches.length === 1
        ) {
          const t = e.changedTouches[0];
          const rect = this.canvas.getBoundingClientRect();
          const node = pickNode(
            this.vp,
            this.visibleNodes(),
            t.clientX - rect.left,
            t.clientY - rect.top,
          );
          if (node) this.clickHandler(node);
        }
      },
      { passive: false },
    );

    this.canvas.addEventListener("touchcancel", () => {
      this.touchPanLast = null;
      this.pinchPrev = null;
      this.poseStart = null;
      this.poseEnd = null;
      this.poseStartCanvas = null;
      this.poseEndCanvas = null;
    });
  }

  // commitPoseIfReady는 위치추정을 확정하기 위해 mouse-up과 touch-end
  // 양쪽에서 호출된다. 5cm 최소 드래그를 강제하고, 다중 클라이언트
  // 미러가 같은 이미지 콘텐츠에 안착하도록 정규화 캔버스 좌표(0..1)를
  // 월드 좌표와 함께 emit한다.
  private commitPoseIfReady(): void {
    if (!this.poseMode || !this.poseStart || !this.poseEnd || !this.poseStartCanvas || !this.poseEndCanvas) {
      this.poseStart = null;
      this.poseEnd = null;
      this.poseStartCanvas = null;
      this.poseEndCanvas = null;
      return;
    }
    const dx = this.poseEnd.x - this.poseStart.x;
    const dy = this.poseEnd.y - this.poseStart.y;
    const dragLen = Math.hypot(dx, dy);
    if (dragLen >= 0.05 && this.poseCommitCb) {
      const theta = Math.atan2(dy, dx);
      const { width, height } = this.canvas;
      const nx = this.poseStartCanvas.cx / width;
      const ny = this.poseStartCanvas.cy / height;
      const dxN = (this.poseEndCanvas.cx - this.poseStartCanvas.cx) / width;
      const dyN = (this.poseEndCanvas.cy - this.poseStartCanvas.cy) / height;
      this.poseCommitCb(this.poseStart.x, this.poseStart.y, theta, nx, ny, dxN, dyN);
    }
    this.poseStart = null;
    this.poseEnd = null;
    this.poseStartCanvas = null;
    this.poseEndCanvas = null;
  }
}
