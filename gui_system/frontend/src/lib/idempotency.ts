// newRequestId는 백엔드의 idempotency cache(gateway/idempotency.go)를
// 구동하는 UUID 형태의 X-Request-ID 헤더를 만든다. 형식과 유일성은 "이
// 세션의 재시도에 충분"하면 된다 — crypto.randomUUID는 우리가 지원하는
// 모든 브라우저에서 사용 가능하다.
export function newRequestId(): string {
  if (typeof crypto !== "undefined" && "randomUUID" in crypto) {
    return crypto.randomUUID();
  }
  // 아주 오래된 환경용 폴백. 대시보드 세션에는 충분하다 — 재부팅 간
  // 충돌은 문제되지 않는다.
  return Math.random().toString(36).slice(2) + Date.now().toString(36);
}
