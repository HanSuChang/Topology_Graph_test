// gui_main은 GUI 백엔드의 진입점이다. 의도적으로 구체 구현들을 하나로
// 연결하는 얇은 조립자다 — 설정, 로깅, 인증, 감사, 브릿지 클라이언트,
// websocket hub, 상태 cache, idempotency cache, SQLite + 리포지토리,
// ETA estimator, API 핸들러, 마지막으로 Gin 엔진.
package main

import (
	"context"
	"flag"
	"log/slog"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/api"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/api/data_analytics"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/api/navigation_map"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/audit"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/bridge"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/core"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/database"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/gateway"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/missionrunner"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/topology"
)

func main() {
	configPath := flag.String("config", "./configs/config.yaml", "path to config.yaml")
	flag.Parse()

	logger := slog.New(slog.NewJSONHandler(os.Stdout, &slog.HandlerOptions{Level: slog.LevelInfo}))

	cfg, err := core.LoadConfig(*configPath)
	if err != nil {
		logger.Error("load config", slog.String("err", err.Error()))
		os.Exit(1)
	}

	nodesFile, err := topology.Load(cfg.NodesFile)
	if err != nil {
		logger.Warn("load nodes file (continuing without topology)", slog.String("err", err.Error()))
		nodesFile = &topology.File{}
	}

	auditLog, err := audit.NewLogger(cfg.Audit.Path)
	if err != nil {
		logger.Error("audit logger", slog.String("err", err.Error()))
		os.Exit(1)
	}
	defer auditLog.Close()

	db, err := database.Open(cfg.DB.Path, cfg.DB.WAL, cfg.MigrationsDir)
	if err != nil {
		logger.Error("db open", slog.String("err", err.Error()))
		os.Exit(1)
	}
	defer db.Close()

	missionRepo := database.NewMissionRepository(db)
	telemetryRepo := database.NewTelemetryRepository(db)

	br, err := bridge.New(cfg.Bridge.Type, cfg.Bridge.Address)
	if err != nil {
		logger.Error("bridge init", slog.String("err", err.Error()))
		os.Exit(1)
	}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	if err := br.Start(ctx); err != nil {
		logger.Warn("bridge start failed (continuing in disconnected state)", slog.String("err", err.Error()))
	}

	cache := gateway.NewCache()
	idem := gateway.NewIdempotencyCache(2 * time.Minute)
	go func() {
		t := time.NewTicker(30 * time.Second)
		defer t.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-t.C:
				idem.Sweep()
			}
		}
	}()

	batch := database.NewBatchWriter(telemetryRepo)
	go batch.Run(ctx)
	defer batch.Stop()

	go data_analytics.RunBackupLoop(ctx, db, cfg.DB.BackupDir, cfg.DB.BackupRetentionDays, logger)

	estimator := data_analytics.NewEstimator(missionRepo, nodesFile.Edges, cfg.ETA.SwarmCorrectionSeconds, cfg.ETA.MinSamplesForAverage, cfg.ETA.DefaultPickSeconds, cfg.ETA.DefaultDropSeconds)

	srv := core.NewServer(cfg, logger, auditLog)
	srv.Health.BridgeProbe = br.Connected
	srv.Health.DBReady.Store(true)

	hub := gateway.NewHub(logger)
	runner := missionrunner.New(logger, func(ev missionrunner.Event) {
		hub.BroadcastEnvelope("client_log", map[string]interface{}{
			"level":     ev.Level,
			"source":    ev.Source,
			"message":   ev.Message,
			"client_id": "backend",
			"ts":        time.Now().UnixMilli(),
		})
	})
	fanout := gateway.NewFanout(br, hub, cache)
	// 텔레메트리를 라이브 송출과 함께 DB 배치 writer로도 흘려보낸다 — 분석
	// 탭 속도 차트가 이 telemetry 테이블을 집계한다.
	fanout.SetTelemetrySink(func(t domain.TelemetryData) {
		batch.Push(database.Row{
			RobotID:   t.RobotID,
			Timestamp: t.Timestamp,
			Velocity:  t.Velocity,
			Battery:   t.Battery,
		})
	})
	fanout.Run(ctx)

	apiH := &api.Handlers{
		Auth:      srv.Auth,
		Audit:     auditLog,
		Bridge:    br,
		Cache:     cache,
		Idem:      idem,
		DB:        db,
		Missions:  missionRepo,
		Estimator: estimator,
		Runner:    runner,
		Nodes:     nodesFile.Nodes,
		Edges:     nodesFile.Edges,
		Hub:       hub,
	}
	navH := navigation_map.NewHandlers(nodesFile.Nodes, nodesFile.Edges, apiH.Robots, cfg.MapYAML)
	analyticsH := data_analytics.NewHandlers(missionRepo, telemetryRepo, estimator)

	core.RegisterRoutes(srv, core.Routes{
		Mission:   apiH,
		Status:    apiH,
		Logs:      apiH,
		Settings:  apiH,
		NavMap:    navH,
		Analytics: analyticsH,
	}, hub)
	core.ServeStatic(srv.Engine(), cfg.Server.StaticDir)

	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)
	go func() {
		<-sigCh
		logger.Info("shutdown requested")
		cancel()
	}()

	if err := srv.Run(ctx); err != nil {
		logger.Error("server exited", slog.String("err", err.Error()))
		os.Exit(1)
	}
	br.Stop()
}
