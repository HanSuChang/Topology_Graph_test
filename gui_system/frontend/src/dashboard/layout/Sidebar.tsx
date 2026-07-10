import { NavLink } from "react-router-dom";

// 설정 탭이 TopStatusBar의 로그인 버튼으로 합쳐진 뒤 최상위 라우트는
// 둘만 남는다. 대시보드는 모든 실시간 제어를 담고, 분석은 오프라인 뷰다.
const TABS = [
  { to: "/", label: "대시보드" },
  { to: "/분석", label: "분석" },
];

export function NavTabs() {
  return (
    <nav className="flex gap-1">
      {TABS.map((t) => (
        <NavLink
          key={t.to}
          to={t.to}
          end={t.to === "/"}
          className={({ isActive }) =>
            `px-3 py-1.5 rounded-lg text-sm font-medium transition-colors ${
              isActive
                ? "bg-accent text-white shadow"
                : "text-muted hover:bg-white/60"
            }`
          }
        >
          {t.label}
        </NavLink>
      ))}
    </nav>
  );
}

// 하위 호환 alias.
export const Sidebar = NavTabs;
