import { useRef, useState, useEffect, useLayoutEffect } from "react";
import { Card, Badge } from "@/dashboard/layout/Card";
import { subscribe as subscribeLogs, type LogRecord } from "@/lib/eventLog";

// LogsStrip은 대시보드 하단의 통합 실시간 로그를 렌더한다. 레코드는 이벤트
// 버스에서 newest-first로 도착한다. tail-of-log 모델을 위해 상한 슬라이스
// (최근 80개)를 뒤집고, 새 항목마다 맨 아래로 자동 스크롤한다.
export function LogsStrip() {
  const [rows, setRows] = useState<LogRecord[]>([]);
  const scrollRef = useRef<HTMLDivElement | null>(null);
  useEffect(() => subscribeLogs(setRows), []);
  useLayoutEffect(() => {
    const el = scrollRef.current;
    if (!el) return;
    el.scrollTop = el.scrollHeight;
  }, [rows]);
  // DOM은 최근 80개 항목으로 제한한다 — 더 오래된 줄은 메모리에 남지만
  // (eventLog는 300개 유지) emit마다 300개 div 트리를 렌더하면 폰에서
  // 페인트 시간이 크게 든다.
  const ordered = rows.slice(0, 80).reverse();
  return (
    <Card title="실시간 로그" className="shrink-0">
      {/* `overflow-auto`가 양축 스크롤을 제공한다; 각 행은
          `whitespace-nowrap min-w-max`라 카드보다 넓은 줄을 좌우로 스와이프할
          수 있다. 자동 스크롤 effect는 scrollTop만 건드리므로 가로 위치는
          보존된다. */}
      <div ref={scrollRef} className="font-mono text-[12px] lg:text-[11px] space-y-0.5 h-32 lg:h-24 overflow-auto">
        {ordered.length === 0 && <div className="text-muted">로그 대기 중…</div>}
        {ordered.map((r) => (
          <div key={r.id} className="flex gap-2 items-center whitespace-nowrap min-w-max">
            <span className="text-muted tabular-nums">{new Date(r.ts).toLocaleTimeString()}</span>
            <Badge tone={r.level === "ERROR" ? "err" : r.level === "WARN" ? "warn" : "muted"}>{r.level}</Badge>
            <span className="text-muted">[{r.source}]</span>
            <span>{r.message}</span>
          </div>
        ))}
      </div>
    </Card>
  );
}
