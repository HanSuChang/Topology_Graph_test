import { useState } from "react";
import { Card } from "@/dashboard/layout/Card";
import { useMission } from "@/hooks/useMission";
import { api } from "@/lib/api";
import { useCommandGuard } from "./hooks";
import { MissionCommandDialog } from "./MissionCommandDialog";

const NODE_OPTIONS = ["storage_a", "storage_b", "delivery_1", "charging", "home"];

// 설계 §5-1 / §2에 따라:
//   - Start / Pause / Resume / Reset / Goal 변경 → 관리자 인증 필요
//   - 긴급 정지         → 인증 없음, 즉시
// useCommandGuard가 busy + 401 → 인증 다이얼로그 흐름을 중앙화한다.
export default function MissionControlPage() {
  const mission = useMission();
  const [target, setTarget] = useState("storage_a");
  const { busy, msg, needsAuth, dismissAuth, setMsg, run } = useCommandGuard();

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
      <Card title="Goal">
        <div className="flex gap-2 mb-3">
          <select
            value={target}
            onChange={(e) => setTarget(e.target.value)}
            className="bg-slate-900 px-3 py-2 rounded border border-slate-700 flex-1"
          >
            {NODE_OPTIONS.map((n) => <option key={n} value={n}>{n}</option>)}
          </select>
          <button
            disabled={busy}
            className="px-3 py-2 rounded bg-accent text-white"
            onClick={() => run(() => api.startMission({ target_node: target }))}
          >
            Start
          </button>
        </div>
        <div className="grid grid-cols-3 gap-2">
          <button disabled={busy} className="px-3 py-2 rounded bg-slate-700" onClick={() => run(api.pause)}>Pause</button>
          <button disabled={busy} className="px-3 py-2 rounded bg-slate-700" onClick={() => run(api.resume)}>Resume</button>
          <button disabled={busy} className="px-3 py-2 rounded bg-slate-700" onClick={() => run(api.reset)}>Reset</button>
        </div>
      </Card>
      <Card title="Emergency">
        <button
          disabled={busy}
          className="w-full py-6 rounded bg-rose-700 hover:bg-rose-600 text-white text-xl font-bold"
          onClick={() => run(api.emergencyStop)}
        >
          EMERGENCY STOP
        </button>
        <button
          disabled={busy}
          className="w-full mt-2 py-2 rounded bg-slate-700"
          onClick={() => run(api.emergencyClear)}
        >
          Clear Emergency (admin)
        </button>
      </Card>
      <Card title="Status">
        <div>Mission: {mission?.mission_status ?? "—"}</div>
        <div>Goal: {mission?.current_goal_node ?? "—"}</div>
        {msg && <div className="text-sm text-slate-400 mt-2">last: {msg}</div>}
      </Card>
      <MissionCommandDialog open={needsAuth} onClose={dismissAuth} onAuthed={() => { dismissAuth(); setMsg("logged in"); }} />
    </div>
  );
}
