import { useEffect, useState } from "react";
import { api } from "@/lib/api";
import type { MissionRow, EtaSample, TimeSeriesPoint } from "./AnalyticsTypes";
import {
  MissionHistoryTable,
  EtaPredictionCard,
  MissionTimeChart,
  SpeedChart,
  FormationErrorChart,
} from "./charts";

// AnalyticsPage는 네 개의 차트와 미션 이력 테이블을 담는다. 모든 데이터는
// /api/v1/missions와 /api/v1/analytics/chart에서 가져오며, 시계열
// 엔드포인트는 텔레메트리가 쌓이면 채워진다.
export default function AnalyticsPage() {
  const [rows, setRows] = useState<MissionRow[]>([]);
  const [chart, setChart] = useState<{ speed: TimeSeriesPoint[]; mission_time: TimeSeriesPoint[]; formation_error: TimeSeriesPoint[] }>({
    speed: [],
    mission_time: [],
    formation_error: [],
  });

  useEffect(() => {
    api.missionHistory().then(setRows).catch(() => setRows([]));
    fetch("/api/v1/analytics/chart").then((r) => r.ok ? r.json() : null).then((c) => {
      if (c) setChart(c);
    }).catch(() => {});
  }, []);

  const eta: EtaSample[] = rows
    .filter((r) => r.predicted_eta != null && r.actual_eta != null)
    .map((r) => ({ id: r.id.slice(0, 8), predicted: r.predicted_eta!, actual: r.actual_eta! }));

  // 데스크탑은 뷰포트 높이를 꽉 채우는 2행 그리드(상단: ETA+이력, 하단: 3차트)로
  // 하단 여백을 없앤다. 모바일은 단일 컬럼 스택 + 페이지 스크롤.
  return (
    <div className="grid grid-cols-12 gap-2 lg:h-full lg:grid-rows-[1.15fr_1fr]">
      <div className="col-span-12 lg:col-span-8 lg:min-h-0">
        <EtaPredictionCard data={eta} />
      </div>
      <div className="col-span-12 lg:col-span-4 lg:min-h-0">
        <MissionHistoryTable rows={rows} />
      </div>
      <div className="col-span-12 lg:col-span-4 lg:min-h-0"><SpeedChart data={chart.speed} /></div>
      <div className="col-span-12 lg:col-span-4 lg:min-h-0"><MissionTimeChart data={chart.mission_time} /></div>
      <div className="col-span-12 lg:col-span-4 lg:min-h-0"><FormationErrorChart data={chart.formation_error} /></div>
    </div>
  );
}
