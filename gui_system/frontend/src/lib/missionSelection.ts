// 미션 제어 드롭다운에서 고른 목적지를 네비게이션 맵과 공유한다.
// 아직 미션을 시작하지 않아 backend mission_state가 오지 않은 상태에서도
// 운영자가 A/B/충전소 선택에 맞는 노드만 미리 볼 수 있게 한다.

type Listener = (target: string) => void;

let target = "loading";
const listeners = new Set<Listener>();

export function currentMissionTarget(): string {
  return target;
}

export function setMissionTarget(next: string): void {
  target = next;
  listeners.forEach((l) => l(target));
}

export function subscribeMissionTarget(l: Listener): () => void {
  listeners.add(l);
  l(target);
  return () => {
    listeners.delete(l);
  };
}
