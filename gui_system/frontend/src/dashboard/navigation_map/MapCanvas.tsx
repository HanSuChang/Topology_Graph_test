import { useEffect, useRef } from "react";
import { MapRenderer } from "./MapRenderer";
import type { MapNode } from "./mapTypes";

// MapCanvas는 명령형 <canvas> + MapRenderer 쌍을 소유해 페이지 컴포넌트가
// 선언적으로 유지되게 한다. 캔버스는 ResizeObserver로 부모에 맞춰 크기를
// 조정하므로, 통합 대시보드 grid와 향후 전체화면 페이지 양쪽에서 동일하게
// 동작한다.
export function MapCanvas({
  onReady,
  onNodeClick,
}: {
  onReady: (renderer: MapRenderer) => void;
  onNodeClick: (node: MapNode) => void;
}) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    const canvas = canvasRef.current!;
    const sync = () => {
      canvas.width = canvas.clientWidth;
      canvas.height = canvas.clientHeight;
    };
    sync();
    const renderer = new MapRenderer(canvas);
    renderer.onNodeClick(onNodeClick);
    onReady(renderer);
    const ro = new ResizeObserver(sync);
    ro.observe(canvas);
    return () => {
      ro.disconnect();
      renderer.destroy();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  return (
    <canvas
      ref={canvasRef}
      // `touch-none`(touch-action: none)은 브라우저가 한 손가락 드래그를
      // 페이지 스크롤로, 두 손가락 제스처를 native 줌으로 가로채는 것을
      // 막는다; 그것들은 MapRenderer가 직접 처리한다. 인라인
      // `transform: translateZ(0)`는 캔버스를 자체 합성 레이어로 승격시켜
      // 30fps 재드로우 루프가 뒤의 glass 카드를 재합성하지 않게 한다.
      className="w-full h-full min-h-[300px] rounded-lg border border-line bg-white/40 block touch-none"
      style={{ transform: "translateZ(0)", willChange: "transform" }}
    />
  );
}
