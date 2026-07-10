import { Card, Badge } from "@/dashboard/layout/Card";
import { useRobotState } from "@/hooks/useRobotState";
import { ROBOT_COLOR, robotRank, isFollower, batteryColor, translateStatus, statusBadgeClass, robotLabel } from "./shared";

// RobotStatusCard는 로봇별 칩을 담는다. 각 칩은 한 줄로 압축된 행이다:
// 점 · id · 좌표 · 배터리 · (추종) · 연결. 순서는 고정: leader 먼저, 그다음
// follower_1, follower_2, … 모바일에서는 칩이 두 줄로 wrap되고 데스크탑에선
// 한 줄을 유지한다.
export function RobotStatusCard() {
  const robots = useRobotState();
  const ordered = robots.slice().sort((a, b) => robotRank(a.robot_id) - robotRank(b.robot_id));
  return (
    <Card title="로봇 상태" className="flex-1 min-h-0 overflow-hidden">
      <div className="flex flex-col gap-1.5">
        {ordered.length === 0 && (
          <span className="px-2 py-1 rounded-md bg-white/75 border border-line text-[12px] text-muted">
            로봇 데이터 없음
          </span>
        )}
        {ordered.map((r) => (
          <div
            key={r.robot_id}
            className="flex items-center gap-1 px-1.5 py-1 rounded-md bg-white/80 border border-line text-[14px] lg:text-[12px] shadow-sm overflow-hidden flex-wrap lg:flex-nowrap lg:whitespace-nowrap"
          >
            <span
              className="w-2.5 h-2.5 rounded-full shrink-0"
              style={{ backgroundColor: ROBOT_COLOR[r.robot_id] ?? "#94a3b8" }}
            />
            <span className="font-medium truncate min-w-0 max-w-[120px] lg:max-w-[76px]">{robotLabel(r.robot_id)}</span>
            <span className="text-muted tabular-nums shrink-0">
              ({r.pose.x.toFixed(1)},{r.pose.y.toFixed(1)})
            </span>
            {/* 배터리·상태(주행)·추종·연결은 한 그룹으로 묶어 ml-auto로 줄 끝에 우측정렬. */}
            <div className="flex items-center gap-1 ml-auto shrink-0">
              {/* RC카 팔로워는 battery_state publisher가 없어 항상 0%로 표시되므로 숨긴다. */}
              {r.robot_id !== "rc_car_follower" && (
                <span className={`tabular-nums shrink-0 font-semibold ${batteryColor(r.battery)}`}>{(r.battery * 100).toFixed(0)}%</span>
              )}
              {r.status && (
                <span className={statusBadgeClass(r.status)}>{translateStatus(r.status)}</span>
              )}
              {/* 역할 뱃지 — 팔로워는 "추종", 리더는 "리더". 리더에도 같은 슬롯에
                  뱃지를 둬 아래 팔로워 행과 열(주행·역할·연결)이 정렬되게 한다. */}
              {isFollower(r.robot_id) ? (
                <span className="shrink-0 px-1 py-0 rounded text-[12px] lg:text-[11px] font-medium border bg-emerald-100/80 text-emerald-700 border-emerald-200">
                  추종
                </span>
              ) : (
                <span className="shrink-0 px-1 py-0 rounded text-[12px] lg:text-[11px] font-medium border bg-indigo-100/80 text-indigo-700 border-indigo-200">
                  리더
                </span>
              )}
              <span
                className={`shrink-0 px-1 py-0 rounded text-[12px] lg:text-[11px] font-medium border ${
                  r.connection_state === "online"
                    ? "bg-emerald-100/80 text-emerald-700 border-emerald-200"
                    : "bg-rose-100/80 text-rose-700 border-rose-200"
                }`}
              >
                {r.connection_state === "online" ? "연결" : "끊김"}
              </span>
            </div>
          </div>
        ))}
        <div className="flex justify-between text-[13px] pt-1 mt-1 border-t border-line/40">
          <span className="text-muted">적재 여부</span>
          <Badge tone="muted">미확인</Badge>
        </div>
      </div>
    </Card>
  );
}
