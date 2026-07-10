import type { MapPoint } from "./mapTypes";

// SlamTransform은 SLAM 맵이 로드됐을 때 world 좌표 ↔ 화면 픽셀 매핑이
// 맵 이미지의 실제 변환(crop → CCW90 회전 → stretch-fit)을 그대로 따르게
// 하는 기하 파라미터다. 백엔드 /map/info가 노출하는 값과 1:1로 대응한다.
// 이 값들이 있으면 노드·로봇·경로·클릭이 모두 맵 벽 위 올바른 위치에 안착한다.
export interface SlamTransform {
  anchorScale: number; // 이 scale에서 맵이 캔버스를 정확히 채움(drawSlam과 동일 기준)
  originX: number;     // map.yaml origin x (world, m)
  originY: number;     // map.yaml origin y (world, m)
  res: number;         // m/픽셀
  origH: number;       // 원본 PGM 높이(행)
  minX: number;        // crop bbox 좌상단 col(원본 픽셀)
  minY: number;        // crop bbox 좌상단 row(원본 픽셀)
  cw: number;          // crop 폭(회전 전)
  ch: number;          // crop 높이(회전 전)
}

// 캔버스 viewport 상태. 렌더러가 이것 하나를 소유하고, 레이어는 참조로
// 받아 toScreen()을 호출해 ROS 월드 좌표(미터, +y 위)를 화면 픽셀
// (+y 아래)로 변환한다. slam이 설정되면 변환은 맵 이미지 파이프라인을
// 따르고, 없으면(맵 미로드) 월드 원점을 캔버스 중앙에 두는 단순 변환을 쓴다.
export interface Viewport {
  scale: number;     // 미터당 픽셀
  originX: number;   // pan 오프셋, 픽셀 단위
  originY: number;
  width: number;     // 캔버스 픽셀 크기
  height: number;
  slam?: SlamTransform;
}

export function newViewport(width: number, height: number): Viewport {
  return { scale: 80, originX: 0, originY: 0, width, height };
}

// stretchBox는 SLAM 이미지가 캔버스에 그려지는 사각형(x0,y0,w,h)을 계산한다.
// MapRenderer.drawSlam의 계산과 정확히 동일해야 노드/로봇이 맵과 함께
// pan·zoom된다.
function stretchBox(vp: Viewport, s: SlamTransform) {
  const k = vp.scale / s.anchorScale;
  const w = vp.width * k;
  const h = vp.height * k;
  const x0 = (vp.width / 2) * (1 - k) + vp.originX;
  const y0 = (vp.height / 2) * (1 - k) + vp.originY;
  return { x0, y0, w, h };
}

export function toScreen(vp: Viewport, p: MapPoint): MapPoint {
  const s = vp.slam;
  if (s) {
    // world → 원본 PGM 픽셀 (ROS: 행 0 = 위, origin = 좌하단).
    const ox = (p.x - s.originX) / s.res;
    const oy = s.origH - 1 - (p.y - s.originY) / s.res;
    // crop bbox 기준 좌표.
    const cx = ox - s.minX;
    const cy = oy - s.minY;
    // CCW90 회전: 처리 이미지 col = cy(폭 ch), row = (cw-1)-cx(높이 cw).
    const nx = cy / s.ch;
    const ny = (s.cw - 1 - cx) / s.cw;
    const { x0, y0, w, h } = stretchBox(vp, s);
    return { x: x0 + nx * w, y: y0 + ny * h };
  }
  const cx = vp.width / 2 + vp.originX;
  const cy = vp.height / 2 + vp.originY;
  return { x: cx + p.x * vp.scale, y: cy - p.y * vp.scale };
}

// headingToScreenAngle은 world heading(theta)을 화면 픽셀 공간의 각도로
// 변환한다. SLAM 모드의 toScreen은 맵의 CCW90 회전·anisotropic stretch를
// 포함하므로, 단순 cos/sin(world theta)을 화면에 그리면 방향이 틀어진다.
// heading 방향의 두 world 점을 toScreen으로 투영해 실제 화면 방향을 얻는다
// (toScreen은 affine이라 offset 크기와 무관). 맵 미로드(단순 변환) 시에는
// 기존 동작(-theta)과 동일하다. 반환 각도는 화면 좌표계(+y 아래) 기준이므로
// 그리는 쪽은 cos/+sin을 그대로 쓰면 된다.
export function headingToScreenAngle(vp: Viewport, p: MapPoint, theta: number): number {
  const tail = toScreen(vp, p);
  const head = toScreen(vp, { x: p.x + Math.cos(theta), y: p.y + Math.sin(theta) });
  return Math.atan2(head.y - tail.y, head.x - tail.x);
}

export function fromScreen(vp: Viewport, px: number, py: number): MapPoint {
  const s = vp.slam;
  if (s) {
    const { x0, y0, w, h } = stretchBox(vp, s);
    const nx = (px - x0) / w;
    const ny = (py - y0) / h;
    // toScreen의 역변환: nx = cy/ch, ny = (cw-1-cx)/cw.
    const cy = nx * s.ch;
    const cx = s.cw - 1 - ny * s.cw;
    const ox = cx + s.minX;
    const oy = cy + s.minY;
    const wx = s.originX + ox * s.res;
    const wy = s.originY + (s.origH - 1 - oy) * s.res;
    return { x: wx, y: wy };
  }
  const cx = vp.width / 2 + vp.originX;
  const cy = vp.height / 2 + vp.originY;
  return { x: (px - cx) / vp.scale, y: (cy - py) / vp.scale };
}
