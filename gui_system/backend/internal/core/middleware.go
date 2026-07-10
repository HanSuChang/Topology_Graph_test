package core

import (
	"log/slog"
	"net"
	"net/http"
	"sync"
	"time"

	"github.com/gin-gonic/gin"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/audit"
)

// StructuredLogger는 요청마다 method, path, status, duration, remote IP를
// 담은 slog.Info를 한 줄 출력한다. 설계 §8-2 로그 형태에 맞춰 같은 writer를
// HTTP + 모듈 로그에 함께 쓸 수 있게 한다.
func StructuredLogger(logger *slog.Logger) gin.HandlerFunc {
	return func(c *gin.Context) {
		start := time.Now()
		c.Next()
		logger.Info("http",
			slog.String("method", c.Request.Method),
			slog.String("path", c.Request.URL.Path),
			slog.Int("status", c.Writer.Status()),
			slog.Duration("dur", time.Since(start)),
			slog.String("ip", c.ClientIP()),
		)
	}
}

// IPAllowList는 긴급 정지가 인증 없이, 단 로컬 LAN에서만 도달 가능하다는
// 설계 §2 규칙을 강제한다.
type IPAllowList struct {
	nets []*net.IPNet
}

func NewIPAllowList(cidrs []string) *IPAllowList {
	out := &IPAllowList{}
	for _, c := range cidrs {
		_, n, err := net.ParseCIDR(c)
		if err == nil {
			out.nets = append(out.nets, n)
		}
	}
	return out
}

func (l *IPAllowList) Allowed(ip string) bool {
	parsed := net.ParseIP(ip)
	if parsed == nil {
		return false
	}
	for _, n := range l.nets {
		if n.Contains(parsed) {
			return true
		}
	}
	return false
}

// RateLimiter는 긴급 정지 앞단에서 쓰는 IP별 분당 카운터다. 정상
// 상황에서는 의도적으로 관대하며(기본 30/분) 명백한 남용만 차단한다.
type RateLimiter struct {
	mu      sync.Mutex
	counts  map[string]int
	windows map[string]time.Time
	limit   int
}

func NewRateLimiter(perMinute int) *RateLimiter {
	if perMinute <= 0 {
		perMinute = 30
	}
	return &RateLimiter{
		counts:  map[string]int{},
		windows: map[string]time.Time{},
		limit:   perMinute,
	}
}

func (r *RateLimiter) Allow(ip string) bool {
	r.mu.Lock()
	defer r.mu.Unlock()
	now := time.Now()
	if w, ok := r.windows[ip]; !ok || now.Sub(w) > time.Minute {
		r.windows[ip] = now
		r.counts[ip] = 0
	}
	r.counts[ip]++
	return r.counts[ip] <= r.limit
}

// EmergencyMiddleware는 긴급 정지 엔드포인트를 IP allow-list와 rate
// limiter로 게이트한다. 설계 §2에 따라 엔드포인트 자체는 비인증으로
// 두며, 이 미들웨어가 유일한 검사다.
func EmergencyMiddleware(allow *IPAllowList, limiter *RateLimiter, log *audit.Logger) gin.HandlerFunc {
	return func(c *gin.Context) {
		ip := c.ClientIP()
		if !allow.Allowed(ip) {
			_ = log.Write(audit.Entry{IP: ip, Command: "EmergencyStop", Result: "blocked_ip"})
			c.AbortWithStatusJSON(http.StatusForbidden, gin.H{"error": "ip not allowed"})
			return
		}
		if !limiter.Allow(ip) {
			_ = log.Write(audit.Entry{IP: ip, Command: "EmergencyStop", Result: "rate_limited"})
			c.AbortWithStatusJSON(http.StatusTooManyRequests, gin.H{"error": "rate limited"})
			return
		}
		c.Next()
	}
}
