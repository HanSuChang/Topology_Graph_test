import { useEffect, useState } from "react";
import { api } from "@/lib/api";
import { wsClient } from "@/lib/websocket";
import type { SystemState } from "@/types/domain/systemState";

// useMission은 SystemState의 미션 레벨 조각을 노출한다: 상태, goal, ETA,
// 긴급 플래그. 로봇 리스트는 useRobotState에 있고, 이 훅은 의도적으로
// 무시해 리렌더가 미션 UI로 한정되게 한다.
export function useMission(): Omit<SystemState, "robots"> | null {
  const [state, setState] = useState<SystemState | null>(null);

  useEffect(() => {
    let cancelled = false;
    const pull = () => {
      api.stateCurrent().then((s) => {
        if (!cancelled) setState(s);
      }).catch(() => {});
    };
    pull();
    const ws = wsClient();
    ws.onReconnect(pull);
    const offMission = ws.on("mission_state", (e) => {
      // 브릿지/백엔드 mission_state는 StatusEvent.payload에 미션 컨텍스트를 담는다.
      const p = (e.payload as any)?.payload ?? {};
      setState((prev) =>
        prev && {
          ...prev,
          mission_status: p.status ?? prev.mission_status,
          current_goal_node: p.current_goal_node ?? prev.current_goal_node,
          eta_seconds: typeof p.eta_seconds === "number" ? p.eta_seconds : prev.eta_seconds,
        },
      );
    });
    return () => { cancelled = true; offMission?.(); };
  }, []);

  if (!state) return null;
  const { robots: _robots, ...rest } = state;
  return rest;
}
