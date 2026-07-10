package gateway

import (
	"sync"
	"time"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// IdempotencyCache는 최근 명령의 결과를 request UUID 키로 저장한다.
// 설계 §3-9에 따라 네트워크 끊김 후 같은 UUID를 재전송하면 명령을
// 두 번 실행(예: 미션 중복 시작)하지 않고 캐시된 CommandResult를
// 반환해야 한다.
type IdempotencyCache struct {
	mu      sync.Mutex
	entries map[string]idemEntry
	ttl     time.Duration
}

type idemEntry struct {
	result    domain.CommandResult
	expiresAt time.Time
}

func NewIdempotencyCache(ttl time.Duration) *IdempotencyCache {
	if ttl == 0 {
		ttl = 2 * time.Minute
	}
	return &IdempotencyCache{entries: map[string]idemEntry{}, ttl: ttl}
}

func (c *IdempotencyCache) Lookup(id string) (domain.CommandResult, bool) {
	if id == "" {
		return domain.CommandResult{}, false
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	e, ok := c.entries[id]
	if !ok {
		return domain.CommandResult{}, false
	}
	if time.Now().After(e.expiresAt) {
		delete(c.entries, id)
		return domain.CommandResult{}, false
	}
	return e.result, true
}

func (c *IdempotencyCache) Store(id string, result domain.CommandResult) {
	if id == "" {
		return
	}
	c.mu.Lock()
	c.entries[id] = idemEntry{result: result, expiresAt: time.Now().Add(c.ttl)}
	c.mu.Unlock()
}

func (c *IdempotencyCache) Sweep() {
	now := time.Now()
	c.mu.Lock()
	for k, e := range c.entries {
		if now.After(e.expiresAt) {
			delete(c.entries, k)
		}
	}
	c.mu.Unlock()
}
