package core

import (
	"crypto/rand"
	"encoding/hex"
	"net/http"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
)

type session struct {
	id        string
	expiresAt time.Time
}

type AuthManager struct {
	mu           sync.RWMutex
	sessions     map[string]session
	passwordHash string
	ttl          time.Duration
}

func NewAuthManager(passwordHash string, ttlMinutes int) *AuthManager {
	if ttlMinutes <= 0 {
		ttlMinutes = 60
	}
	return &AuthManager{
		sessions:     map[string]session{},
		passwordHash: passwordHash,
		ttl:          time.Duration(ttlMinutes) * time.Minute,
	}
}

func (a *AuthManager) Login(passwordHash string) (string, bool) {
	if a.passwordHash == "" {
		// dev 모드: 비밀번호 미설정, 어떤 로그인이든 수락
	} else if a.passwordHash != passwordHash {
		return "", false
	}
	buf := make([]byte, 16)
	_, _ = rand.Read(buf)
	id := hex.EncodeToString(buf)
	a.mu.Lock()
	a.sessions[id] = session{id: id, expiresAt: time.Now().Add(a.ttl)}
	a.mu.Unlock()
	return id, true
}

func (a *AuthManager) Validate(id string) bool {
	a.mu.RLock()
	s, ok := a.sessions[id]
	a.mu.RUnlock()
	if !ok {
		return false
	}
	if time.Now().After(s.expiresAt) {
		a.mu.Lock()
		delete(a.sessions, id)
		a.mu.Unlock()
		return false
	}
	return true
}

func (a *AuthManager) Logout(id string) {
	a.mu.Lock()
	delete(a.sessions, id)
	a.mu.Unlock()
}

// RequireAuth는 쓰기 엔드포인트를 게이트하는 Gin 미들웨어다.
// 설계 §2에 따라 읽기 엔드포인트(대시보드, 상태, 차트)는 인증이 없고,
// 명령 엔드포인트는 관리자 세션을 요구한다. 자체 미들웨어를 쓰는 긴급
// 정지만 유일한 예외다.
func (a *AuthManager) RequireAuth() gin.HandlerFunc {
	return func(c *gin.Context) {
		id, err := c.Cookie("session_id")
		if err != nil || !a.Validate(id) {
			c.AbortWithStatusJSON(http.StatusUnauthorized, gin.H{"error": "auth required"})
			return
		}
		c.Set("session_id", id)
		c.Next()
	}
}
