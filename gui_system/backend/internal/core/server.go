package core

import (
	"context"
	"fmt"
	"log/slog"
	"net"
	"net/http"
	"time"

	"github.com/gin-contrib/cors"
	"github.com/gin-gonic/gin"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/audit"
)

// Server는 Gin 엔진을 보조 보안 헬퍼(auth manager, allow-list, rate
// limiter) 및 다른 goroutine들이 브릿지/DB 장애를 감지하며 뒤집는
// HealthState와 함께 묶는다. 이 형태 덕에 cmd/gui_main이 의존성을 한
// 곳에서 연결할 수 있다.
type Server struct {
	cfg    *Config
	logger *slog.Logger
	engine *gin.Engine
	srv    *http.Server

	Auth    *AuthManager
	Audit   *audit.Logger
	Allow   *IPAllowList
	Limiter *RateLimiter
	Health  *HealthState
}

func NewServer(cfg *Config, logger *slog.Logger, auditLog *audit.Logger) *Server {
	gin.SetMode(gin.ReleaseMode)
	engine := gin.New()
	engine.Use(gin.Recovery())
	engine.Use(StructuredLogger(logger))

	corsCfg := cors.DefaultConfig()
	corsCfg.AllowAllOrigins = true
	corsCfg.AllowHeaders = []string{"Origin", "Content-Type", "Authorization", "X-Request-ID"}
	corsCfg.AllowCredentials = false
	engine.Use(cors.New(corsCfg))

	return &Server{
		cfg:     cfg,
		logger:  logger,
		engine:  engine,
		Auth:    NewAuthManager(cfg.Auth.AdminPasswordHash, cfg.Auth.SessionTTLMinutes),
		Audit:   auditLog,
		Allow:   NewIPAllowList(cfg.Emergency.AllowedCIDRs),
		Limiter: NewRateLimiter(cfg.Emergency.RateLimitPerMinute),
		Health:  &HealthState{},
	}
}

func (s *Server) Engine() *gin.Engine { return s.engine }

func (s *Server) Config() *Config { return s.cfg }

// lanURLs는 비-루프백 IPv4 인터페이스를 열거해 host 브라우저(예 윈도우
// Chrome)에서 접근할 만한 URL 목록을 만든다. VirtualBox bridged 어댑터의
// 게스트 IP가 여기 잡혀 운영자에게 그대로 노출된다.
func lanURLs(port int) []string {
	ifaces, err := net.Interfaces()
	if err != nil {
		return nil
	}
	var urls []string
	for _, iface := range ifaces {
		if iface.Flags&net.FlagUp == 0 || iface.Flags&net.FlagLoopback != 0 {
			continue
		}
		addrs, err := iface.Addrs()
		if err != nil {
			continue
		}
		for _, a := range addrs {
			ipnet, ok := a.(*net.IPNet)
			if !ok {
				continue
			}
			ip := ipnet.IP.To4()
			if ip == nil || ip.IsLoopback() || ip.IsLinkLocalUnicast() {
				continue
			}
			urls = append(urls, fmt.Sprintf("http://%s:%d", ip.String(), port))
		}
	}
	return urls
}

func (s *Server) Run(ctx context.Context) error {
	addr := fmt.Sprintf("%s:%d", s.cfg.Server.Host, s.cfg.Server.Port)
	s.srv = &http.Server{Addr: addr, Handler: s.engine}

	errCh := make(chan error, 1)
	go func() {
		s.logger.Info("server listening", slog.String("addr", addr))
		// 호스트 브라우저(윈도우 Chrome 등)에서 바로 붙을 수 있도록 LAN
		// IPv4 URL을 함께 출력. 0.0.0.0 바인드일 때만 의미가 있다.
		s.logger.Info("open in browser",
			slog.String("local", fmt.Sprintf("http://localhost:%d", s.cfg.Server.Port)),
			slog.Any("lan", lanURLs(s.cfg.Server.Port)),
		)
		if err := s.srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			errCh <- err
		}
	}()

	select {
	case <-ctx.Done():
		grace := time.Duration(s.cfg.Server.GracefulShutdownSeconds) * time.Second
		if grace == 0 {
			grace = 5 * time.Second
		}
		shutCtx, cancel := context.WithTimeout(context.Background(), grace)
		defer cancel()
		return s.srv.Shutdown(shutCtx)
	case err := <-errCh:
		return err
	}
}
