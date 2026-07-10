import { useEffect, useRef, useState } from "react";
import { createPortal } from "react-dom";
import { api } from "@/lib/api";

// 관리자 게이트 제어 전반이 쓰는 인증 다이얼로그. `auth.admin_password_hash`가
// 빈 값(dev 모드)인 동안 백엔드는 어떤 비밀번호든 수락하므로, 현재는
// 주로 /missions/*가 쓰는 세션 쿠키를 만드는 흐름 게이트다. 다이얼로그는
// fixed/inset-0 + flex 패턴으로 중앙 정렬되며, 제출은 Enter로 발생한다
// (form 요소 + button[type=submit])라 키보드 흐름이 RViz 및 운영자의
// 나머지 도구와 일치한다.
export function MissionCommandDialog({
  open,
  onClose,
  onAuthed,
}: {
  open: boolean;
  onClose: () => void;
  onAuthed: () => void;
}) {
  const [username, setUsername] = useState("admin");
  const [pw, setPw] = useState("");
  const [err, setErr] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const firstFieldRef = useRef<HTMLInputElement | null>(null);

  // 다이얼로그가 열릴 때마다 아이디 필드에 자동 포커스해 운영자가 바로
  // 타이핑할 수 있게 한다.
  useEffect(() => {
    if (open) {
      setErr(null);
      // input이 DOM에 존재하도록 포커스를 다음 tick으로 미룬다.
      setTimeout(() => firstFieldRef.current?.focus(), 0);
    }
  }, [open]);

  // Esc로 닫기 — 나머지 모달 관용구와 일치.
  useEffect(() => {
    if (!open) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") onClose();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [open, onClose]);

  if (!open) return null;
  // `position: fixed`가 뷰포트 기준으로 해석되도록 <body>로 렌더한다.
  // 이게 없으면 backdrop-filter를 쓰는 주변 `.glass` 조상이 containing
  // block이 되어 다이얼로그를 카드 크기 영역으로 클리핑한다 — 그러면
  // input이 화면 밖이나 다른 패널 아래로 가 타이핑이 막힌다.
  if (typeof document === "undefined") return null;

  const submit = async () => {
    if (busy) return;
    setBusy(true);
    setErr(null);
    try {
      await api.login(pw, username);
      setPw("");
      onAuthed();
    } catch (e: any) {
      setErr(e?.message ?? "로그인 실패");
    } finally {
      setBusy(false);
    }
  };

  return createPortal(
    <div
      className="fixed inset-0 z-50 bg-black/40 backdrop-blur-sm flex items-center justify-center"
      onMouseDown={(e) => {
        // 어두운 백드롭 클릭은 다이얼로그를 닫는다; 카드 내부 클릭은
        // 아래의 stop-propagation으로 차단된다.
        if (e.target === e.currentTarget) onClose();
      }}
    >
      <form
        className="glass rounded-2xl p-6 w-[320px] border border-line shadow-xl"
        onMouseDown={(e) => e.stopPropagation()}
        onSubmit={(e) => {
          e.preventDefault();
          void submit();
        }}
      >
        <h3 className="text-base font-bold text-ink mb-3">관리자 로그인</h3>

        <label className="block text-[11px] text-muted mb-1">아이디</label>
        <input
          ref={firstFieldRef}
          type="text"
          autoComplete="username"
          value={username}
          onChange={(e) => setUsername(e.target.value)}
          className="w-full bg-white/80 border border-line rounded-lg px-3 py-2 text-sm mb-3 focus:outline-none focus:ring-2 focus:ring-accent/40"
          placeholder="관리자 ID"
        />

        <label className="block text-[11px] text-muted mb-1">비밀번호</label>
        <input
          type="password"
          autoComplete="current-password"
          value={pw}
          onChange={(e) => setPw(e.target.value)}
          className="w-full bg-white/80 border border-line rounded-lg px-3 py-2 text-sm mb-3 focus:outline-none focus:ring-2 focus:ring-accent/40"
          placeholder="비밀번호"
        />

        {err && (
          <div className="text-rose-600 bg-rose-50 border border-rose-200 rounded-md px-2 py-1 text-xs mb-3">
            {err}
          </div>
        )}

        <div className="flex gap-2 justify-end">
          <button
            type="button"
            className="px-3 py-1.5 rounded-lg border border-line bg-white/70 hover:bg-white text-sm"
            onClick={onClose}
          >
            취소
          </button>
          <button
            type="submit"
            disabled={busy}
            className="px-3 py-1.5 rounded-lg bg-accent text-white text-sm shadow hover:brightness-110 disabled:opacity-50"
          >
            {busy ? "로그인 중…" : "로그인"}
          </button>
        </div>

        <p className="text-[10px] text-muted/70 mt-3">
          ↵ Enter 로 로그인 · Esc 로 취소
        </p>
      </form>
    </div>,
    document.body,
  );
}
