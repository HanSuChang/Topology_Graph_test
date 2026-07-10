// 대시보드용 통합 in-process 로그 버스. 로컬 UI 이벤트(버튼 클릭)와 원격
// WS 이벤트 모두 `emit()`을 거쳐 LogsStrip이 단일 진실 원천을 갖게 한다.
// 순서는 newest-first.
//
// 다중 클라이언트 미러: 로컬 emit은 /api/v1/client_log로도 POST되어 백엔드가
// 다른 모든 대시보드 클라이언트로 줄을 fan-out한다. 수신한 `client_log`
// envelope은 네트워크 왕복을 건너뛰는 `emitLocal`로 다시 흘러간다 — 이것이
// 루프 차단기다. 클라이언트 식별자는 탭별 UUID이며, 수신 측은 자기
// client_id를 걸러내 원본 클라이언트가 중복을 보지 않게 한다.

export type LogLevel = "INFO" | "WARN" | "ERROR";

export interface LogRecord {
  id: number;
  ts: number;
  level: LogLevel;
  source: string;
  message: string;
}

type Listener = (records: LogRecord[]) => void;

let counter = 0;
const records: LogRecord[] = [];
const listeners = new Set<Listener>();
const MAX = 300;

// 탭별 식별자. 페이지 로드마다 재생성된다 — 영속 세션 상태를 추적하지
// 않고도 "이건 내 로그" 중복을 피하기에 충분하다.
const CLIENT_ID =
  typeof crypto !== "undefined" && "randomUUID" in crypto
    ? crypto.randomUUID()
    : `c_${Math.random().toString(36).slice(2)}_${Date.now()}`;

export function clientId(): string {
  return CLIENT_ID;
}

// emitLocal은 네트워크 전파 없이 in-memory ring에 넣고 구독자에게 알린다.
// WS 수신부와 emit() 자신이 사용한다.
function emitLocal(level: LogLevel, source: string, message: string, ts?: number): void {
  counter += 1;
  records.unshift({
    id: counter,
    ts: ts ?? Date.now(),
    level,
    source,
    message,
  });
  if (records.length > MAX) records.length = MAX;
  const snap = records.slice();
  listeners.forEach((l) => l(snap));
}

// WebSocket으로 다른 클라이언트의 로그를 수신한다. client_id가 자기 것과
// 같으면(서버 echo) 건너뛴다 — 아니면 원본 클라이언트가 모든 줄을 두 번
// 보게 된다.
export function emitFromRemote(record: {
  level: LogLevel;
  source: string;
  message: string;
  client_id?: string;
  ts?: number;
}): void {
  if (record.client_id && record.client_id === CLIENT_ID) return;
  emitLocal(record.level, record.source, record.message, record.ts);
}

// 공개 emit — 로컬에 넣은 뒤 다중 클라이언트 미러를 발사한다. 네트워크
// 실패는 조용히 삼키며, 로컬 항목은 그대로 표시되므로 불안정한 네트워크가
// UX 피드백을 막지 않는다.
export function emit(level: LogLevel, source: string, message: string): void {
  const ts = Date.now();
  emitLocal(level, source, message, ts);
  // 최선 노력 broadcast. `keepalive: true`는 빠른 네비게이션에도 요청이
  // 살아남게 한다(Safari는 내부적으로 sendBeacon처럼 취급). 오류는
  // 의도적으로 삼킨다.
  try {
    fetch("/api/v1/client_log", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      keepalive: true,
      body: JSON.stringify({
        level,
        source,
        message,
        ts,
        client_id: CLIENT_ID,
      }),
    }).catch(() => {});
  } catch {
    /* fetch 불가 — 로컬 전용 폴백 */
  }
}

export function subscribe(listener: Listener): () => void {
  listeners.add(listener);
  listener(records.slice());
  return () => {
    listeners.delete(listener);
  };
}

// 헬퍼 — 호출부를 짧고 일관되게 유지.
export const logInfo = (source: string, msg: string) => emit("INFO", source, msg);
export const logWarn = (source: string, msg: string) => emit("WARN", source, msg);
export const logErr  = (source: string, msg: string) => emit("ERROR", source, msg);
