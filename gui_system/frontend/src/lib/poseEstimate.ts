// "위치추정" 토글을 위한 단순 pub/sub. 버튼 카드와 네비게이션 맵은 서로
// 다른 React 서브트리에 있지만, 운영자가 현재 초기 pose를 찍는 중인지에
// 합의해야 한다. 이 모듈이 공유 진실 원천이다.

type Listener = (active: boolean) => void;

let active = false;
const listeners = new Set<Listener>();

export function isActive(): boolean {
  return active;
}

export function setActive(v: boolean): void {
  active = v;
  listeners.forEach((l) => l(active));
}

export function toggle(): boolean {
  setActive(!active);
  return active;
}

export function subscribe(l: Listener): () => void {
  listeners.add(l);
  l(active);
  return () => {
    listeners.delete(l);
  };
}
