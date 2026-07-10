import { PropsWithChildren } from "react";
import { TopStatusBar } from "./TopStatusBar";

// DashboardLayout은 의도적으로 단순하다: 상단 헤더, 아래에 빠듯한 패딩의
// 스크롤 가능한 페이지 영역. 대시보드 페이지 자체는 빽빽한 grid라 운영자가
// 스크롤 없이 모든 것을 본다.
export function DashboardLayout({ children }: PropsWithChildren) {
  return (
    <div className="flex flex-col h-full">
      <TopStatusBar />
      <main className="flex-1 overflow-auto px-3 py-3">{children}</main>
    </div>
  );
}
