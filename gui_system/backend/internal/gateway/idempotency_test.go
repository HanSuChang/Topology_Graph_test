package gateway

import (
	"testing"
	"time"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

func TestIdempotencyLookup(t *testing.T) {
	c := NewIdempotencyCache(time.Second)
	if _, ok := c.Lookup("x"); ok {
		t.Fatalf("unexpected hit on empty cache")
	}
	c.Store("x", domain.CommandResult{RequestID: "x", Accepted: true})
	if r, ok := c.Lookup("x"); !ok || !r.Accepted {
		t.Fatalf("expected cached result")
	}
}

func TestIdempotencyExpiry(t *testing.T) {
	c := NewIdempotencyCache(20 * time.Millisecond)
	c.Store("x", domain.CommandResult{RequestID: "x", Accepted: true})
	time.Sleep(40 * time.Millisecond)
	if _, ok := c.Lookup("x"); ok {
		t.Fatalf("expected expired entry to be gone")
	}
}
