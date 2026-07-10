// 경량 알림 버스. 컴포넌트가 구독하고, 미션 제어나 WebSocket alert
// 핸들러가 토스트를 push한다. Should 단계 shadcn Toast 통합이 끼워넣기로
// 가능하도록 어떤 UI 라이브러리에도 의존하지 않는다.

type Severity = "info" | "warn" | "error";
export interface Notice {
  id: number;
  ts: number;
  severity: Severity;
  text: string;
}

type Listener = (notices: Notice[]) => void;

let counter = 0;
const notices: Notice[] = [];
const listeners = new Set<Listener>();

export function push(severity: Severity, text: string) {
  counter += 1;
  notices.unshift({ id: counter, ts: Date.now(), severity, text });
  if (notices.length > 50) notices.length = 50;
  listeners.forEach((l) => l([...notices]));
}

export function subscribe(l: Listener): () => void {
  listeners.add(l);
  l([...notices]);
  return () => listeners.delete(l);
}
