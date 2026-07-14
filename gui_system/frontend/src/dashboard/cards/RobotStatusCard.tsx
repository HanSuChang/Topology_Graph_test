import { Card, Badge } from "@/dashboard/layout/Card";
import { useMission } from "@/hooks/useMission";
import { useRobotState } from "@/hooks/useRobotState";
import { ROBOT_COLOR, robotRank, isFollower, batteryColor, translateStatus, statusBadgeClass, robotLabel } from "./shared";

// RobotStatusCard는 로봇별 칩을 담는다. 각 칩은 한 줄로 압축된 행이다:
// 점 · id · 좌표 · 배터리 · (추종) · 연결. 순서는 고정: leader 먼저, 그다음
// follower_1, follower_2, … 모바일에서는 칩이 두 줄로 wrap되고 데스크탑에선
// 한 줄을 유지한다.
export function RobotStatusCard() {
  const robots = useRobotState();
  const mission = useMission();
  const ordered = robots.slice().sort((a, b) => robotRank(a.robot_id) - robotRank(b.robot_id));
  const workStatus = mission?.mission_status;
  const workLabel =
    workStatus === "completed" ? "작업 완료" :
    workStatus === "running" ? "작업중" :
    workStatus === "paused" ? "일시정지" :
    workStatus === "failed" || workStatus === "aborted" ? "실패" :
    "대기";
  const workTone: "ok" | "warn" | "err" | "muted" | "accent" =
    workStatus === "completed" ? "ok" :
    workStatus === "running" ? "accent" :
    workStatus === "paused" ? "warn" :
    workStatus === "failed" || workStatus === "aborted" ? "err" :
    "muted";
  return (
    <Card title="로봇 상태" className="flex-1 min-h-0 overflow-hidden">
      <div className="flex flex-col gap-1.5">
        {ordered.length === 0 && (
          <span className="px-2 py-1 rounded-md bg-white/75 border border-line text-[12px] text-muted">
            로봇 데이터 없음
          </span>
        )}
        {ordered.map((r) => (
          <div key={r.robot_id} className="rounded-md bg-white/80 border border-line px-2 py-1.5 text-[12px] shadow-sm">
            <div className="flex items-center gap-1.5 min-w-0">
              <span
                className="w-2.5 h-2.5 rounded-full shrink-0"
                style={{ backgroundColor: ROBOT_COLOR[r.robot_id] ?? "#94a3b8" }}
              />
              <span className="font-semibold truncate min-w-0">{robotLabel(r.robot_id)}</span>
              <span className="ml-auto text-muted tabular-nums shrink-0">
                ({r.pose.x.toFixed(1)}, {r.pose.y.toFixed(1)})
              </span>
            </div>
            <div className="mt-1 flex items-center gap-1 overflow-hidden">
              {r.status && <span className={statusBadgeClass(r.status)}>{translateStatus(r.status)}</span>}
              <span className="shrink-0 px-1 py-0 rounded text-[11px] font-medium border bg-slate-100/80 text-slate-600 border-slate-200 tabular-nums">
                v {(r.velocity ?? 0).toFixed(2)}
              </span>
              {r.robot_id !== "rc_car_follower" && (
                <span className={`shrink-0 tabular-nums font-semibold ${batteryColor(r.battery)}`}>{(r.battery * 100).toFixed(0)}%</span>
              )}
              <div className="ml-auto flex items-center gap-1 shrink-0">
                {isFollower(r.robot_id) ? (
                  <span className="shrink-0 px-1 py-0 rounded text-[11px] font-medium border bg-emerald-100/80 text-emerald-700 border-emerald-200">
                    추종
                  </span>
                ) : (
                  <span className="shrink-0 px-1 py-0 rounded text-[11px] font-medium border bg-indigo-100/80 text-indigo-700 border-indigo-200">
                    리더
                  </span>
                )}
                <span
                  className={`shrink-0 px-1 py-0 rounded text-[11px] font-medium border ${
                    r.connection_state === "online"
                      ? "bg-emerald-100/80 text-emerald-700 border-emerald-200"
                      : "bg-rose-100/80 text-rose-700 border-rose-200"
                  }`}
                >
                  {r.connection_state === "online" ? "연결" : "끊김"}
                </span>
              </div>
            </div>
          </div>
        ))}
        <div className="flex justify-between text-[13px] pt-1 mt-1 border-t border-line/40">
          <span className="text-muted">작업 상태</span>
          <Badge tone={workTone}>{workLabel}</Badge>
        </div>
      </div>
    </Card>
  );
}
