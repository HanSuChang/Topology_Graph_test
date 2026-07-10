package api

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/audit"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/core"
)

// Login은 관리자 세션을 수립한다. dev 모드(admin_password_hash가 빈 값)
// 에서는 자격 증명을 프로비저닝하지 않고도 데모를 돌릴 수 있도록 어떤
// 비밀번호든 수락한다.
func (h *Handlers) Login(c *gin.Context) {
	var body struct {
		PasswordHash string `json:"password_hash"`
		Username     string `json:"username,omitempty"`
	}
	if err := c.ShouldBindJSON(&body); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "bad body"})
		return
	}
	auth, _ := c.MustGet("auth").(*core.AuthManager)
	id, ok := auth.Login(body.PasswordHash)
	if !ok {
		_ = h.Audit.Write(audit.Entry{IP: c.ClientIP(), Command: "Login", Result: "denied", Detail: "user=" + body.Username})
		c.JSON(http.StatusUnauthorized, gin.H{"error": "invalid credentials"})
		return
	}
	c.SetCookie("session_id", id, 3600, "/", "", false, true)
	_ = h.Audit.Write(audit.Entry{IP: c.ClientIP(), Command: "Login", Result: "accepted", Detail: "user=" + body.Username})
	c.JSON(http.StatusOK, gin.H{"session_id": id})
}
