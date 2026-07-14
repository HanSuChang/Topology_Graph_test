import { useEffect, useState } from "react";
import { Card, Badge } from "@/dashboard/layout/Card";
import { wsClient } from "@/lib/websocket";

type ManipulatorState = {
  statusText: string;
  angles: number[] | null;
  lastUpdate: number | null;
};

function gripperLabel(angles: number[] | null): string {
  if (!angles || angles.length < 5) return "미확인";
  return angles[4] >= 115 ? "열림" : "닫힘";
}

function statusTone(text: string): "ok" | "warn" | "err" | "muted" | "accent" {
  const t = text.toLowerCase();
  if (!text) return "muted";
  if (t.includes("fail") || t.includes("error")) return "err";
  if (t.includes("waiting") || t.includes("no target") || t.includes("not detected")) return "warn";
  if (t.includes("mission start") || t.includes("auto") || t.includes("servo")) return "accent";
  if (t.includes("complete") || t.includes("finish")) return "ok";
  return "muted";
}

// ManipulatorCard는 암의 현재 자세 + 그리퍼 상태를 보여준다. 복구
// 제어(gripper open/close/home)는 관리자 인증으로 게이트되며 Should
// 단계에서 들어온다.
export function ManipulatorCard() {
  const [state, setState] = useState<ManipulatorState>({
    statusText: "",
    angles: null,
    lastUpdate: null,
  });

  useEffect(() => {
    const ws = wsClient();
    const off = ws.on("alert", (e) => {
      const eventType = e.payload?.event_type;
      const payload = e.payload?.payload ?? {};
      if (eventType === "arm_vision_status") {
        setState((prev) => ({
          ...prev,
          statusText: String(payload.status_text ?? ""),
          lastUpdate: Date.now(),
        }));
      }
      if (eventType === "arm_servo_angles" && Array.isArray(payload.angles)) {
        setState((prev) => ({
          ...prev,
          angles: payload.angles.map((v: unknown) => Number(v)).filter(Number.isFinite),
          lastUpdate: Date.now(),
        }));
      }
    });
    return () => {
      off?.();
    };
  }, []);

  const angles = state.angles;
  const updated = state.lastUpdate ? `${Math.max(0, Math.round((Date.now() - state.lastUpdate) / 1000))}초 전` : "—";

  return (
    <Card title="매니퓰레이터 상태" className="shrink-0">
      <div className="flex gap-1.5 mb-2">
        <Badge tone={statusTone(state.statusText)}>{state.statusText || "대기"}</Badge>
        <Badge tone="muted">그리퍼: {gripperLabel(angles)}</Badge>
      </div>
      <div className="text-xs space-y-0.5 text-muted">
        <div className="tabular-nums">
          서보: {angles ? angles.map((v, i) => `S${i}:${v}`).join("  ") : "—"}
        </div>
        <div>최근 상태: {state.statusText || "—"}</div>
        <div>최근 갱신: {updated}</div>
      </div>
      <div className="text-[11px] text-muted/70 mt-2">
        로봇팔 토픽을 읽기 전용으로 표시합니다.
      </div>
    </Card>
  );
}
