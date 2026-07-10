import { PropsWithChildren, ReactNode } from "react";

// Card는 모든 페이지가 재사용하는 유일한 표면 프리미티브다. 비주얼은
// 본문 그라데이션 위로 떠오르는 라이트 글래스 타일(반투명 흰색 + backdrop
// blur)이다. 소제목은 노트북 화면에서도 1m 거리에서 또렷이 읽히도록
// 크기를 잡았다.

export function Card({
  title,
  action,
  className = "",
  children,
}: PropsWithChildren<{ title?: string; action?: ReactNode; className?: string }>) {
  return (
    <section className={`glass rounded-xl p-2 sm:p-3 ${className}`}>
      {(title || action) && (
        <header className="flex items-center justify-between mb-2">
          {title && (
            <h2 className="text-base font-bold tracking-tight text-ink">
              {title}
            </h2>
          )}
          {action}
        </header>
      )}
      {children}
    </section>
  );
}

export function Badge({
  tone,
  children,
}: PropsWithChildren<{ tone?: "ok" | "warn" | "err" | "muted" | "accent" }>) {
  const cls =
    tone === "ok"     ? "bg-emerald-100/80 text-emerald-700 border-emerald-200" :
    tone === "warn"   ? "bg-amber-100/80 text-amber-700 border-amber-200" :
    tone === "err"    ? "bg-rose-100/80 text-rose-700 border-rose-200" :
    tone === "accent" ? "bg-blue-100/80 text-blue-700 border-blue-200" :
    "bg-slate-100/80 text-slate-600 border-slate-200";
  return (
    <span className={`px-2 py-0.5 rounded-full text-[11px] font-medium border ${cls}`}>
      {children}
    </span>
  );
}
