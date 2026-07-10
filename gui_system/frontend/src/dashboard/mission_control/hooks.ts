import { useState, useCallback } from "react";

// useCommandGuard는 관리자 명령 호출을 감싸 각 버튼이 다음을 할 수 있게 한다:
//   1) 진행 중에는 비활성화(busy)
//   2) 결과/오류를 같은 자리에 표시
//   3) 401 시 인증 다이얼로그 트리거(무엇을 할지는 호출 측이 결정)
export function useCommandGuard() {
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<string | null>(null);
  const [needsAuth, setNeedsAuth] = useState(false);

  const run = useCallback(async (fn: () => Promise<unknown>) => {
    setBusy(true);
    setMsg(null);
    try {
      await fn();
      setMsg("ok");
    } catch (e: any) {
      if (String(e?.message ?? "").startsWith("401")) setNeedsAuth(true);
      else setMsg(String(e?.message ?? "error"));
    } finally {
      setBusy(false);
    }
  }, []);

  return { busy, msg, needsAuth, dismissAuth: () => setNeedsAuth(false), setMsg, run };
}
