import { useEffect } from "react";
import { wsClient } from "@/lib/websocket";

// useReconnect는 WebSocket gateway가 재접속할 때마다 cb를 실행한다.
// 주로 useRobotState/useMission이 끊김 후 reconciliation 스냅샷을 다시
// 가져오는 데 쓴다.
export function useReconnect(cb: () => void) {
  useEffect(() => {
    wsClient().onReconnect(cb);
  }, [cb]);
}
