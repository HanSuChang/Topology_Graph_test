import { useEffect, useState } from "react";
import { subscribe, type Notice } from "@/lib/notifications";

// useNotifications는 lib/notifications.ts를 React 상태로 미러링한다.
// 버스 자체는 plain하게 유지해 비-React 유틸리티(예: WS 오류 핸들러)가
// React를 import하지 않고도 notice를 push할 수 있다.
export function useNotifications(): Notice[] {
  const [notices, setNotices] = useState<Notice[]>([]);
  useEffect(() => subscribe(setNotices), []);
  return notices;
}
