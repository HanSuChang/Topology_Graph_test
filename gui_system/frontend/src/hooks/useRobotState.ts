import { useEffect, useRef, useState } from "react";
import { api } from "@/lib/api";
import { wsClient } from "@/lib/websocket";
import type { RobotState } from "@/types/domain/robotState";

// 로봇이 이 시간 이상 아무 데이터도 보내지 않으면 연결이 끊긴 것으로 본다.
// telemetry가 ~2Hz로 흐르므로 5초면 정상 동작 중 오탐 없이 전원 OFF를 잡는다.
const STALE_MS = 5000;

// useRobotState는 robot_pose envelope을 구독해 id별 map에 병합하므로 React가
// 변경된 카드만 렌더한다. 초기값은 /state/robots에서 오며, WS 재접속마다
// 갱신된다. 또한 수신이 끊긴(전원 OFF) 로봇을 stale로 판정해
// connection_state를 offline으로 덮어쓴다 — 마지막 'online' 상태가 영구히
// 남아 꺼진 로봇이 연결됨으로 보이던 문제를 막는다.
export function useRobotState(): RobotState[] {
  const [byId, setById] = useState<Map<string, RobotState>>(new Map());
  // 마지막 수신 시각(robot_pose 또는 telemetry). 메시지마다 re-render하지
  // 않도록 ref에 두고, 아래 1초 ticker가 stale 재평가용 re-render를 유발한다.
  const lastSeen = useRef<Map<string, number>>(new Map());
  const [, setTick] = useState(0);

  useEffect(() => {
    let cancelled = false;
    const mark = (id: string) => lastSeen.current.set(id, Date.now());
    const pull = () => {
      api.stateRobots().then((r) => {
        if (cancelled) return;
        const m = new Map<string, RobotState>();
        const now = Date.now();
        for (const rs of r.robots || []) {
          m.set(rs.robot_id, rs);
          // 초기 로드 시 grace 부여 — 이후 STALE_MS 안에 live 데이터가
          // 안 오면 자연히 offline으로 넘어간다.
          lastSeen.current.set(rs.robot_id, now);
        }
        setById(m);
      }).catch(() => {});
    };
    pull();
    const ws = wsClient();
    ws.onReconnect(pull);
    const offPose = ws.on("robot_pose", (e) => {
      const rs = e.payload as RobotState | undefined;
      if (!rs?.robot_id) return;
      mark(rs.robot_id);
      setById((prev) => {
        const next = new Map(prev);
        next.set(rs.robot_id, rs);
        return next;
      });
    });
    // telemetry는 /odom 기반 ~2Hz로, 정지 중에도 로봇이 켜져 있는 한 계속
    // 흐르므로 pose보다 신뢰할 수 있는 liveness heartbeat다. (amcl_pose는
    // 로봇이 멈춰 있으면 재발행되지 않을 수 있다.)
    const offTel = ws.on("telemetry", (e) => {
      const id = (e.payload?.robot_id ?? e.robot_id) as string | undefined;
      if (id) mark(id);
    });
    // 새 메시지가 없어도 stale 전이를 반영하려면 주기적으로 재평가해야 한다.
    const timer = setInterval(() => setTick((t) => t + 1), 1000);
    return () => {
      cancelled = true;
      offPose?.();
      offTel?.();
      clearInterval(timer);
    };
  }, []);

  const now = Date.now();
  return Array.from(byId.values()).map((rs) => {
    const seen = lastSeen.current.get(rs.robot_id) ?? 0;
    if (now - seen > STALE_MS && rs.connection_state !== "offline") {
      return { ...rs, connection_state: "offline" };
    }
    return rs;
  });
}
