import { useState, useEffect } from "react";
import { Card } from "@/dashboard/layout/Card";
import { logInfo } from "@/lib/eventLog";
import * as poseEstimate from "@/lib/poseEstimate";

// PoseEstimateCard는 RViz 스타일 "2D Pose Estimate" 진입점을 제공한다.
// 버튼을 누르면 네비게이션 맵이 구독하는 전역 모드 플래그가 토글되고,
// 활성화되면 운영자가 맵에서 클릭+드래그로 마커 + 헤딩 화살표를 찍는다.
//
// `className`을 forward해 브레이크포인트별로 카드를 보이거나 숨길 수 있다 —
// 모바일에서는 위치추정을 우측 레일, 요약을 좌측 레일에 두길 원했다
// (데스크탑 레이아웃의 반대).
export function PoseEstimateCard({ className = "" }: { className?: string }) {
  const [active, setActive] = useState(poseEstimate.isActive());
  useEffect(() => poseEstimate.subscribe(setActive), []);
  return (
    <Card title="위치 추정" className={`shrink-0 ${className}`}>
      <button
        className={`w-full py-2 rounded-lg text-sm font-medium border transition shadow-sm ${
          active
            ? "bg-accent text-white border-accent"
            : "bg-white/80 border-line text-ink hover:bg-white"
        }`}
        onClick={() => {
          const next = poseEstimate.toggle();
          logInfo("pose", next ? "위치 추정 모드 활성화" : "위치 추정 모드 해제");
        }}
      >
        {active ? "추정 중… (맵에서 클릭+드래그)" : "위치 추정 시작"}
      </button>
    </Card>
  );
}
