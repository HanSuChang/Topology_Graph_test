import { api } from "./api";
import { wsClient } from "./websocket";
import type { SystemState } from "@/types/domain/systemState";

// reconcile()은 Go 백엔드에서 권위 있는 SystemState를 가져와 setter에
// 넘긴다. 훅은 mount 시 한 번, 그리고 WebSocket 재접속마다 다시 구독하므로
// 끊김 동안 잃은 상태가 REST로 복구된다. 설계 §3-10 / §7-1과 일치.
export function reconcileOnReconnect(setter: (s: SystemState) => void) {
  const ws = wsClient();
  const pull = () => {
    api.stateCurrent().then(setter).catch(() => {});
  };
  pull();
  ws.onReconnect(pull);
  return pull;
}
