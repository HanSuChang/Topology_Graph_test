import type { MessageEnvelope } from "@/types/transport/messageEnvelope";

type Handler = (e: MessageEnvelope) => void;

// 자동 재연결(상한 있는 지수 backoff)을 갖춘 단순 type-dispatch WebSocket
// 클라이언트. 끊김 reconciliation은 호출 측의 몫이다: /state/current를
// 다시 가져오는 onReconnect 콜백을 등록하면 된다.
export class WSClient {
  private ws: WebSocket | null = null;
  private handlers = new Map<string, Set<Handler>>();
  private url: string;
  private retryMs = 1000;
  private alive = true;
  private reconnectCb: (() => void) | null = null;

  constructor(url?: string) {
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    this.url = url ?? `${proto}//${location.host}/ws`;
  }

  on(type: string, h: Handler) {
    if (!this.handlers.has(type)) this.handlers.set(type, new Set());
    this.handlers.get(type)!.add(h);
    return () => this.handlers.get(type)?.delete(h);
  }

  onReconnect(cb: () => void) {
    this.reconnectCb = cb;
  }

  connect() {
    if (!this.alive) return;
    this.ws = new WebSocket(this.url);
    this.ws.onopen = () => {
      this.retryMs = 1000;
      this.reconnectCb?.();
    };
    this.ws.onmessage = (ev) => {
      try {
        const e = JSON.parse(ev.data) as MessageEnvelope;
        this.handlers.get(e.type)?.forEach((h) => h(e));
      } catch {
        /* malformed frame, drop */
      }
    };
    this.ws.onclose = () => {
      if (!this.alive) return;
      setTimeout(() => this.connect(), this.retryMs);
      this.retryMs = Math.min(this.retryMs * 2, 15000);
    };
    this.ws.onerror = () => this.ws?.close();
  }

  close() {
    this.alive = false;
    this.ws?.close();
  }
}

let _client: WSClient | null = null;
export function wsClient() {
  if (!_client) {
    _client = new WSClient();
    _client.connect();
  }
  return _client;
}
