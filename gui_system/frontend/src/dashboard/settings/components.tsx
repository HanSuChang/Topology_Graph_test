import { useEffect, useState } from "react";
import { Card, Badge } from "@/dashboard/layout/Card";
import { api } from "@/lib/api";
import type { HealthResponse } from "@/types/api/response";

export function AdminLogin() {
  const [pw, setPw] = useState("");
  const [msg, setMsg] = useState<string | null>(null);
  return (
    <Card title="관리자 로그인">
      <input
        type="password"
        value={pw}
        onChange={(e) => setPw(e.target.value)}
        placeholder="비밀번호 (개발 모드에서는 임의값)"
        className="w-full bg-white/80 border border-line rounded-lg px-3 py-2 text-sm mb-2"
      />
      <button
        className="px-3 py-1.5 rounded-lg bg-accent text-white text-sm shadow"
        onClick={async () => {
          try {
            await api.login(pw);
            setMsg("로그인 완료");
          } catch (e: any) {
            setMsg(e.message);
          }
        }}
      >
        로그인
      </button>
      {msg && <div className="text-xs text-muted mt-2">{msg}</div>}
    </Card>
  );
}

export function BridgeSettings() {
  const [h, setH] = useState<HealthResponse | null>(null);
  useEffect(() => {
    api.health().then(setH).catch(() => setH(null));
  }, []);
  return (
    <Card title="브릿지">
      <div className="text-sm flex items-center gap-2">
        브릿지 상태
        <Badge tone={h?.bridge === "connected" ? "ok" : "err"}>
          {h?.bridge === "connected" ? "연결됨" : (h?.bridge ?? "—")}
        </Badge>
      </div>
      <div className="text-xs text-muted mt-2">
        Bridge 종류/주소는 <code>backend/configs/config.yaml</code>에서 변경합니다.
      </div>
    </Card>
  );
}

export function NodeConfigPanel() {
  return (
    <Card title="토폴로지">
      <div className="text-xs text-muted">
        노드 / 엣지는 <code>maps/nodes.yaml</code>에서 정의됩니다.
        자유 좌표 입력은 허용되지 않습니다 (설계 §6-2).
      </div>
    </Card>
  );
}
