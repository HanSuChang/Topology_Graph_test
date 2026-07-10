import { useEffect } from "react";
import { wsClient } from "@/lib/websocket";
import type { MessageEnvelope } from "@/types/transport/messageEnvelope";

// useWebSocket은 하나의 envelope 타입을 구독하고 프레임마다 cb를 실행한다.
// unmount 시 자동으로 구독 해제한다.
export function useWebSocket(type: string, cb: (e: MessageEnvelope) => void) {
  useEffect(() => {
    const off = wsClient().on(type, cb);
    return () => {
      off?.();
    };
  }, [type, cb]);
}
