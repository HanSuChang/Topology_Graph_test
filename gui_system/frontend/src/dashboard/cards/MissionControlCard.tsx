import { useEffect, useState } from "react";
import { Card } from "@/dashboard/layout/Card";
import { api } from "@/lib/api";
import { logInfo, logErr } from "@/lib/eventLog";
import { useCommandGuard } from "@/dashboard/mission_control/hooks";
import { MissionCommandDialog } from "@/dashboard/mission_control/MissionCommandDialog";
import { setMissionTarget } from "@/lib/missionSelection";
import { Btn, NODE_LABEL } from "./shared";

// 운영자가 목적지로 고를 수 있는 노드 타입. 상차 구역 / 리더 정차 슬롯(A·B
// 구역) / 충전소만 노출하고, 교차로·진입·정밀보조 같은 경유 노드는 숨긴다.
const DESTINATION_TYPES = new Set(["loading_zone", "stationing_slot", "charger"]);

// /state/map fetch 실패 시 폴백 목적지(실제 토폴로지 노드 id). 카드가 절대
// 빈 드롭다운이 되지 않게 한다.
const FALLBACK_DESTS: { id: string; label: string }[] = [
  { id: "loading", label: "상차 구역" },
  { id: "a_leader_slot", label: "A 구역" },
  { id: "b_leader_slot", label: "B 구역" },
  { id: "charger_front", label: "충전소" },
];

// MissionControlCard는 운영자의 명령 표면이다: target 노드를 고르고
// 시작/일시정지/재개/초기화, 그리고 긴급 정지·해제를 발행한다. 모든
// 클릭이 `[ui] 요청 → [command] 수락됨/실패`로 로그되어, 브릿지가 조용해도
// 실시간 로그가 동작을 반영한다. 목적지 목록은 토폴로지(/state/map)에서
// 받아 실제 노드 id와 일치시킨다(과거 하드코딩 storage_a 등 제거).
export function MissionControlCard() {
  const [nodes, setNodes] = useState<{ id: string; label: string }[]>(FALLBACK_DESTS);
  const [target, setTarget] = useState(FALLBACK_DESTS[0].id);
  const { busy, msg, needsAuth, dismissAuth, setMsg, run } = useCommandGuard();

  useEffect(() => {
    let cancelled = false;
    api.stateMap().then((m) => {
      if (cancelled) return;
      const dests = (m.nodes ?? [])
        .filter((n: any) => DESTINATION_TYPES.has(n.type) && n.enabled !== false)
        .map((n: any) => {
          const id = n.node_id ?? n.id;
          return { id, label: NODE_LABEL[id] ?? n.name ?? id };
        });
      if (dests.length > 0) {
        setNodes(dests);
        setTarget((prev) => {
          const next = dests.some((d) => d.id === prev) ? prev : dests[0].id;
          setMissionTarget(next);
          return next;
        });
      }
    }).catch(() => {});
    return () => { cancelled = true; };
  }, []);

  const guarded = (label: string, fn: () => Promise<unknown>) =>
    run(async () => {
      logInfo("ui", `${label} 요청`);
      try {
        await fn();
        logInfo("command", `${label} 수락됨`);
      } catch (e: any) {
        logErr("command", `${label} 실패: ${e?.message ?? "오류"}`);
        throw e;
      }
    });

  return (
    <Card title="미션 제어" className="shrink-0">
      <div className="flex gap-2 mb-2">
        <select
          value={target}
          onChange={(e) => {
            setTarget(e.target.value);
            setMissionTarget(e.target.value);
          }}
          className="flex-1 bg-white/80 border border-line rounded-lg px-2 py-1.5 text-sm"
        >
          {nodes.map((n) => <option key={n.id} value={n.id}>{n.label}</option>)}
        </select>
        <button
          disabled={busy}
          className="px-3 py-1.5 rounded-lg bg-accent text-white text-sm shadow hover:brightness-110 disabled:opacity-50"
          onClick={() => guarded(`주행 시작 → ${target}`, () => api.startMission({ target_node: target }))}
        >
          시작
        </button>
      </div>
      <div className="grid grid-cols-3 gap-2 mb-2">
        <Btn disabled={busy} onClick={() => guarded("일시정지", api.pause)}>일시정지</Btn>
        <Btn disabled={busy} onClick={() => guarded("재개", api.resume)}>재개</Btn>
        <Btn disabled={busy} onClick={() => guarded("미션 초기화", api.reset)}>초기화</Btn>
      </div>
      <button
        disabled={busy}
        className="w-full py-3 rounded-lg bg-gradient-to-br from-rose-500 to-rose-600 text-white font-bold tracking-wide shadow hover:brightness-110 disabled:opacity-50"
        onClick={() => guarded("긴급 정지", api.emergencyStop)}
      >
        긴급 정지
      </button>
      <button
        disabled={busy}
        className="w-full mt-1.5 py-1 rounded-lg text-xs text-muted bg-white/60 border border-line hover:bg-white/80"
        onClick={() => guarded("긴급 해제", api.emergencyClear)}
      >
        긴급 해제 (관리자)
      </button>
      {msg && <div className="text-[11px] text-muted mt-2">마지막: {msg}</div>}
      <MissionCommandDialog
        open={needsAuth}
        onClose={dismissAuth}
        onAuthed={() => { dismissAuth(); setMsg("로그인 완료"); logInfo("auth", "관리자 로그인 완료"); }}
      />
    </Card>
  );
}
