package core

import (
	"fmt"
	"os"
	"path/filepath"

	"gopkg.in/yaml.v3"
)

type ServerConfig struct {
	Host                    string `yaml:"host"`
	Port                    int    `yaml:"port"`
	StaticDir               string `yaml:"static_dir"`
	GracefulShutdownSeconds int    `yaml:"graceful_shutdown_seconds"`
}

type BridgeConfig struct {
	Type            string `yaml:"type"`
	Address         string `yaml:"address"`
	ProtocolVersion int    `yaml:"protocol_version"`
}

type DBConfig struct {
	Path                string `yaml:"path"`
	WAL                 bool   `yaml:"wal"`
	BackupDir           string `yaml:"backup_dir"`
	BackupRetentionDays int    `yaml:"backup_retention_days"`
}

type AuthConfig struct {
	AdminPasswordHash string `yaml:"admin_password_hash"`
	SessionTTLMinutes int    `yaml:"session_ttl_minutes"`
}

type EmergencyConfig struct {
	AllowedCIDRs       []string `yaml:"allowed_cidrs"`
	RateLimitPerMinute int      `yaml:"rate_limit_per_minute"`
}

type AuditConfig struct {
	Path string `yaml:"path"`
}

type ETAConfig struct {
	SwarmCorrectionSeconds float64 `yaml:"swarm_correction_seconds"`
	MinSamplesForAverage   int     `yaml:"min_samples_for_average"`
	DefaultPickSeconds     float64 `yaml:"default_pick_seconds"`
	DefaultDropSeconds     float64 `yaml:"default_drop_seconds"`
}

type Config struct {
	Server        ServerConfig    `yaml:"server"`
	Bridge        BridgeConfig    `yaml:"bridge"`
	DB            DBConfig        `yaml:"db"`
	Auth          AuthConfig      `yaml:"auth"`
	Emergency     EmergencyConfig `yaml:"emergency"`
	Audit         AuditConfig     `yaml:"audit"`
	ETA           ETAConfig       `yaml:"eta"`
	NodesFile     string          `yaml:"nodes_file"`
	MapYAML       string          `yaml:"map_yaml"`
	MigrationsDir string          `yaml:"migrations_dir"`
}

func LoadConfig(path string) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("read config %s: %w", path, err)
	}
	var c Config
	if err := yaml.Unmarshal(data, &c); err != nil {
		return nil, fmt.Errorf("parse config: %w", err)
	}
	// 모든 경로형 필드를 config 파일 디렉토리 기준으로 해석한 뒤 절대
	// 경로로 만들어, 어떤 cwd에서 시작하든 바이너리가 동작하게 한다.
	cfgAbs, err := filepath.Abs(path)
	if err != nil {
		return nil, err
	}
	cfgDir := filepath.Dir(cfgAbs)
	resolve := func(p string) string {
		if p == "" || filepath.IsAbs(p) {
			return p
		}
		return filepath.Clean(filepath.Join(cfgDir, p))
	}
	c.NodesFile = resolve(c.NodesFile)
	c.MapYAML = resolve(c.MapYAML)
	c.DB.Path = resolve(c.DB.Path)
	c.DB.BackupDir = resolve(c.DB.BackupDir)
	c.Audit.Path = resolve(c.Audit.Path)
	c.Server.StaticDir = resolve(c.Server.StaticDir)
	c.MigrationsDir = resolve(c.MigrationsDir)
	return &c, nil
}
