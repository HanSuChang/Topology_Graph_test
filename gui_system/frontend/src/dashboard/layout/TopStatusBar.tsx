import { useEffect, useState } from "react";
import { api } from "@/lib/api";
import { logInfo } from "@/lib/eventLog";
import type { HealthResponse } from "@/types/api/response";
import { useMission } from "@/hooks/useMission";
import { Badge } from "./Card";
import { NavTabs } from "./Sidebar";
import { MissionCommandDialog } from "@/dashboard/mission_control/MissionCommandDialog";

// TopStatusBar는 항상 보이는 유일한 chrome이다. 브랜드 마크, 네비게이션
// 탭, 실시간 health 뱃지, 현재 시각, 관리자 로그인 진입점을 담는다(설정
// 탭은 운영자 요청으로 제거됨).
export function TopStatusBar() {
  const [h, setH] = useState<HealthResponse | null>(null);
  const [authOpen, setAuthOpen] = useState(false);
  const mission = useMission();
  useEffect(() => {
    const pull = () => api.health().then(setH).catch(() => setH(null));
    pull();
    const t = setInterval(pull, 3000);
    return () => clearInterval(t);
  }, []);

  const label = (k: string, v?: string) => `${k}: ${v ?? "—"}`;

  return (
    <header className="glass-strong sticky top-0 z-30 flex items-center gap-x-3 gap-y-1 flex-wrap px-3 lg:px-4 py-2 border-b border-line/60">
      <div className="flex items-center gap-2 mr-1 lg:mr-2">
        <div className="w-7 h-7 rounded-lg bg-gradient-to-br from-sky-400 to-violet-500 shadow shrink-0" />
        <span className="font-bold text-base tracking-tight">
          <span className="lg:hidden">물류관제</span>
          <span className="hidden lg:inline">스마트 물류 관제 시스템</span>
        </span>
      </div>
      <NavTabs />
      <div className="flex gap-1.5 ml-0 lg:ml-2 flex-wrap">
        <Badge tone={h?.server === "ok" ? "ok" : "err"}>{label("서버", h?.server ?? "—")}</Badge>
        <Badge tone={h?.bridge === "connected" ? "ok" : "err"}>{label("브릿지", h?.bridge ?? "—")}</Badge>
        <Badge tone={h?.db === "ready" ? "ok" : "err"}>{label("DB", h?.db ?? "—")}</Badge>
      </div>
      <div className="lg:ml-auto flex items-center gap-2 text-xs flex-wrap">
        {mission?.emergency_active && <Badge tone="err">긴급 정지 활성</Badge>}
        <WallClock />
        <button
          className="ml-1 lg:ml-2 px-3 py-1.5 lg:py-1 rounded-lg bg-white/70 hover:bg-white border border-line text-xs font-medium"
          onClick={() => setAuthOpen(true)}
        >
          관리자 로그인
        </button>
      </div>
      <MissionCommandDialog
        open={authOpen}
        onClose={() => setAuthOpen(false)}
        onAuthed={() => {
          setAuthOpen(false);
          logInfo("auth", "관리자 로그인 완료");
        }}
      />
    </header>
  );
}

// WallClock은 1초마다 갱신돼 운영자가 관리자 로그인 버튼 옆에서 항상
// 실제 시각을 본다. 미션 요약 필드는 운영자 요청으로 제거됐다 — 대시보드
// 우측 레일 "요약" 카드에 있다.
function WallClock() {
  const [now, setNow] = useState(() => new Date());
  useEffect(() => {
    const t = setInterval(() => setNow(new Date()), 1000);
    return () => clearInterval(t);
  }, []);
  return (
    <span className="text-muted tabular-nums">
      <b className="text-ink">{now.toLocaleTimeString()}</b>
    </span>
  );
}
