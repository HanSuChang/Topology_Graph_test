import { Route, Routes, Navigate } from "react-router-dom";
import { DashboardLayout } from "@/dashboard/layout/DashboardLayout";
import MainDashboardPage from "@/dashboard/MainDashboardPage";
import AnalyticsPage from "@/dashboard/data_analytics/AnalyticsPage";

// 두 개의 표면: 실시간 대시보드와 오프라인 분석. 관리자 로그인은
// TopStatusBar의 버튼으로 합쳐져 네비게이션에 설정 탭은 더 이상 없다.
export default function App() {
  return (
    <DashboardLayout>
      <Routes>
        <Route path="/" element={<MainDashboardPage />} />
        <Route path="/분석" element={<AnalyticsPage />} />
        <Route path="*" element={<Navigate to="/" replace />} />
      </Routes>
    </DashboardLayout>
  );
}
