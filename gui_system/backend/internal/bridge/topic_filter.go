package bridge

import (
	"sync"
	"time"
)

// TopicFilter는 향후 grpc 또는 websocket 브릿지가 받는 메시지에 설계
// §3-6 다운샘플링 레이트를 적용한다. 각 토픽(pose / odom / tf / costmap)은
// 자체 rate gate를 가지며, gate 허용 윈도우 사이 호출은 드랍된다. Python
// 브릿지가 이미 서버 측에서 다운샘플하므로, 이는 브릿지 구현이 예상보다
// 빠르게 push할 때를 위한 Go 측 back-pressure 레이어다.
type TopicFilter struct {
	mu    sync.Mutex
	gates map[string]time.Time
	rates map[string]time.Duration
}

func NewTopicFilter() *TopicFilter {
	return &TopicFilter{
		gates: map[string]time.Time{},
		rates: map[string]time.Duration{
			"pose":    200 * time.Millisecond, // 5Hz
			"odom":    200 * time.Millisecond,
			"tf":      200 * time.Millisecond,
			"costmap": time.Second,
		},
	}
}

// Allow는 주어진 토픽의 메시지를 forward해도 되면 true를 반환한다.
// rate가 등록되지 않은 토픽은 필터 없이 통과한다.
func (f *TopicFilter) Allow(topic string) bool {
	rate, ok := f.rates[topic]
	if !ok {
		return true
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	now := time.Now()
	if last, seen := f.gates[topic]; seen && now.Sub(last) < rate {
		return false
	}
	f.gates[topic] = now
	return true
}
