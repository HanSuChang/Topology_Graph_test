import { useRef, useState } from "react";
import { Card } from "@/dashboard/layout/Card";
import { api } from "@/lib/api";
import { wsClient } from "@/lib/websocket";
import { MapCanvas } from "./MapCanvas";
import type { MapRenderer } from "./MapRenderer";

// NavigationMapPage는 Canvas 측 MapRenderer를 백엔드 데이터에 연결한다:
//   - 토폴로지는 /api/v1/state/map에서 온다(one-shot)
//   - 로봇 pose는 WebSocket으로 robot_pose envelope으로 도착
//   - 경로는 WebSocket으로 path_data envelope으로 도착
// 노드 클릭은 Goal 후보를 선택한다; "시작" 버튼은 /api/v1/missions/start로
// POST한다(관리자 인증 필요).
export default function NavigationMapPage() {
  const rendererRef = useRef<MapRenderer | null>(null);
  const [selected, setSelected] = useState<string | null>(null);
  const [msg, setMsg] = useState<string | null>(null);

  const onReady = (renderer: MapRenderer) => {
    rendererRef.current = renderer;
    api.stateMap().then((m) => {
      renderer.setTopology(
        (m.nodes ?? []).map((n: any) => ({ id: n.node_id ?? n.id, name: n.name, x: n.x, y: n.y, type: n.type })),
        (m.edges ?? []).map((e: any) => ({ from: e.from_node, to: e.to_node })),
      );
    }).catch(() => {});

    const ws = wsClient();
    ws.on("robot_pose", (e) => {
      const rs = e.payload;
      if (!rs?.robot_id || !rs?.pose) return;
      renderer.updateRobotPose(rs.robot_id, rs.pose.x, rs.pose.y, rs.pose.theta ?? 0);
    });
    ws.on("path_data", (e) => {
      const p = e.payload;
      if (!p?.robot_id) return;
      renderer.updatePath(p.robot_id, p.kind ?? "global", p.points ?? []);
    });
  };

  const sendGoal = async () => {
    if (!selected) return;
    try {
      await api.startMission({ target_node: selected });
      setMsg(`mission start → ${selected}`);
    } catch (e: any) {
      setMsg(e.message);
    }
  };

  return (
    <Card title="Navigation Map">
      <div className="flex items-center gap-3 mb-3">
        <span className="text-sm">선택 노드: <code>{selected ?? "—"}</code></span>
        <button
          disabled={!selected}
          className="px-3 py-1.5 rounded bg-accent text-white disabled:opacity-40"
          onClick={sendGoal}
        >
          Goal로 미션 시작
        </button>
        {msg && <span className="text-xs text-slate-400">{msg}</span>}
      </div>
      <MapCanvas onReady={onReady} onNodeClick={(n) => setSelected(n.id)} />
      <div className="text-xs text-slate-500 mt-2">
        마우스 휠로 줌, 드래그로 이동. 등록된 노드만 목적지로 선택 가능 (설계 §6-2).
      </div>
    </Card>
  );
}
