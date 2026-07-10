// 분석 페이지용 Recharts 기반 카드들. 각 차트는 동일한 LineChart 형태를
// 감싼 20줄짜리 래퍼다; 여기로 합쳐 작은 파일 다섯 개와 미션 이력 테이블을
// 없애면서 차트별 props는 깔끔하고 재사용 가능하게 유지한다.

import { LineChart, Line, AreaChart, Area, XAxis, YAxis, Tooltip, ResponsiveContainer, Legend, CartesianGrid } from "recharts";
import { Card } from "@/dashboard/layout/Card";
import type { EtaSample, MissionRow, TimeSeriesPoint } from "./AnalyticsTypes";

const TOOLTIP_STYLE = {
  background: "rgba(255,255,255,0.9)",
  border: "1px solid #cbd5e1",
  borderRadius: 8,
};

// epoch millis(ts) → "시:분:초". 시계열 차트의 X축 눈금과 툴팁 라벨에 쓴다.
function fmtTime(ts: number): string {
  if (!Number.isFinite(ts)) return "";
  return new Date(ts).toLocaleTimeString("ko-KR", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  });
}

// 공통 시계열 카드. area=true면 점 없는 매끄러운 영역 그래프(밀집 데이터용),
// 아니면 점이 있는 라인(미션처럼 소수 이산 포인트용). X축은 시:분:초.
function TimeSeriesCard({
  title,
  data,
  unit,
  color,
  area = false,
}: {
  title: string;
  data: TimeSeriesPoint[];
  unit: string;
  color: string;
  area?: boolean;
}) {
  const gradId = `grad${color.replace("#", "")}`;
  return (
    <Card title={title} className="h-full flex flex-col">
      <div className="flex-1 min-h-[9rem]">
        <ResponsiveContainer width="100%" height="100%">
          {area ? (
            <AreaChart data={data} margin={{ top: 8, right: 8, bottom: 0, left: 0 }}>
              <defs>
                <linearGradient id={gradId} x1="0" y1="0" x2="0" y2="1">
                  <stop offset="5%" stopColor={color} stopOpacity={0.35} />
                  <stop offset="95%" stopColor={color} stopOpacity={0.02} />
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(148,163,184,0.18)" vertical={false} />
              <XAxis dataKey="ts" stroke="#64748b" fontSize={11} tickFormatter={fmtTime} minTickGap={48} tickMargin={6} />
              <YAxis stroke="#64748b" fontSize={11} width={40} domain={[0, "auto"]} />
              <Tooltip contentStyle={TOOLTIP_STYLE} labelFormatter={(v) => fmtTime(Number(v))} />
              <Area type="monotone" dataKey="value" name={unit} stroke={color} strokeWidth={2} fill={`url(#${gradId})`} dot={false} activeDot={{ r: 3 }} isAnimationActive={false} />
            </AreaChart>
          ) : (
            <LineChart data={data} margin={{ top: 8, right: 8, bottom: 0, left: 0 }}>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(148,163,184,0.18)" vertical={false} />
              <XAxis dataKey="ts" stroke="#64748b" fontSize={11} tickFormatter={fmtTime} minTickGap={48} tickMargin={6} />
              <YAxis stroke="#64748b" fontSize={11} width={40} />
              <Tooltip contentStyle={TOOLTIP_STYLE} labelFormatter={(v) => fmtTime(Number(v))} />
              <Line type="monotone" dataKey="value" name={unit} stroke={color} strokeWidth={2} dot={{ r: 2.5 }} isAnimationActive={false} />
            </LineChart>
          )}
        </ResponsiveContainer>
      </div>
    </Card>
  );
}

export function SpeedChart({ data }: { data: TimeSeriesPoint[] }) {
  return <TimeSeriesCard title="속도" data={data} unit="m/s" color="#06b6d4" area />;
}

export function MissionTimeChart({ data }: { data: TimeSeriesPoint[] }) {
  return <TimeSeriesCard title="미션 소요 시간" data={data} unit="초" color="#f59e0b" />;
}

export function FormationErrorChart({ data }: { data: TimeSeriesPoint[] }) {
  return <TimeSeriesCard title="군집 추종 오차" data={data} unit="m" color="#ec4899" area />;
}

// EtaPredictionCard — 예측 vs 실제 ETA, 나란히 표시되는 라인.
export function EtaPredictionCard({ data }: { data: EtaSample[] }) {
  return (
    <Card title="예상 시간 vs 실제" className="h-full flex flex-col">
      <div className="flex-1 min-h-[9rem]">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data}>
            <XAxis dataKey="id" stroke="#64748b" fontSize={11} />
            <YAxis stroke="#64748b" fontSize={11} />
            <Tooltip contentStyle={TOOLTIP_STYLE} />
            <Legend />
            <Line type="monotone" dataKey="predicted" name="예측(초)" stroke="#2563eb" />
            <Line type="monotone" dataKey="actual" name="실제(초)" stroke="#10b981" />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </Card>
  );
}

// MissionHistoryTable — 상태, target, ETA를 담은 최근 미션 목록.

const STATUS_LABEL: Record<string, string> = {
  pending: "대기",
  running: "진행 중",
  paused: "일시정지",
  completed: "완료",
  failed: "실패",
  aborted: "중단",
};

export function MissionHistoryTable({ rows }: { rows: MissionRow[] }) {
  return (
    <Card title="최근 미션" className="h-full flex flex-col">
      <div className="flex-1 min-h-0 overflow-auto">
      <table className="w-full text-xs">
        {/* 스크롤 시 컬럼 헤더가 카드 상단에 고정되도록 sticky. 각 th에 반투명
            흰 배경을 줘 스크롤되는 행이 헤더 뒤로 비치지 않게 한다. */}
        <thead className="text-muted">
          <tr className="text-left">
            <th className="p-1.5 sticky top-0 bg-white/95 border-b border-line/60">ID</th>
            <th className="p-1.5 sticky top-0 bg-white/95 border-b border-line/60">상태</th>
            <th className="p-1.5 sticky top-0 bg-white/95 border-b border-line/60">목적지</th>
            <th className="p-1.5 text-right sticky top-0 bg-white/95 border-b border-line/60">예상 시간</th>
            <th className="p-1.5 text-right sticky top-0 bg-white/95 border-b border-line/60">실제 시간</th>
          </tr>
        </thead>
        <tbody>
          {rows.length === 0 && (
            <tr><td colSpan={5} className="text-center text-muted py-4">미션 기록 없음</td></tr>
          )}
          {rows.map((r) => (
            <tr key={r.id} className="border-t border-line/40">
              <td className="p-1.5 font-mono">{r.id.slice(0, 8)}</td>
              <td className="p-1.5">{STATUS_LABEL[r.status] ?? r.status}</td>
              <td className="p-1.5">{r.target_node}</td>
              <td className="p-1.5 text-right tabular-nums">{r.predicted_eta?.toFixed(0)}초</td>
              <td className="p-1.5 text-right tabular-nums">{r.actual_eta?.toFixed(0)}초</td>
            </tr>
          ))}
        </tbody>
      </table>
      </div>
    </Card>
  );
}
