package database

import (
	"database/sql"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
)

// RunMigrations는 dir의 모든 .sql 파일을 사전순으로 적용하며, 이미 실행된
// 버전을 `schema_migrations` 테이블에 기록한다. 숫자 파일명 prefix(`001_`,
// `002_`)가 순서를 정한다.
//
// 의도적으로 아주 작게 유지한다: DSL 없음, down-migration 없음, 파일 간
// 트랜잭션 없음. 처음 세 migration이 missions / telemetry / travel times
// 스키마를 만든다 — 그보다 큰 것은 제대로 된 migration 도구에 맡겨야
// 하지만, 단일 바이너리 내장 SQLite에는 이 정도면 충분하다.
func RunMigrations(conn *sql.DB, dir string) error {
	if _, err := conn.Exec(`CREATE TABLE IF NOT EXISTS schema_migrations (
		version TEXT PRIMARY KEY,
		applied_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP
	)`); err != nil {
		return fmt.Errorf("create schema_migrations: %w", err)
	}

	entries, err := os.ReadDir(dir)
	if err != nil {
		return fmt.Errorf("read migrations dir %s: %w", dir, err)
	}
	names := make([]string, 0, len(entries))
	for _, e := range entries {
		if e.IsDir() || !strings.HasSuffix(e.Name(), ".sql") {
			continue
		}
		names = append(names, e.Name())
	}
	sort.Strings(names)

	for _, name := range names {
		version := strings.TrimSuffix(name, ".sql")
		var seen int
		_ = conn.QueryRow(`SELECT COUNT(*) FROM schema_migrations WHERE version=?`, version).Scan(&seen)
		if seen > 0 {
			continue
		}
		body, err := os.ReadFile(filepath.Join(dir, name))
		if err != nil {
			return fmt.Errorf("read %s: %w", name, err)
		}
		if _, err := conn.Exec(string(body)); err != nil {
			return fmt.Errorf("apply %s: %w", name, err)
		}
		if _, err := conn.Exec(`INSERT INTO schema_migrations (version) VALUES (?)`, version); err != nil {
			return fmt.Errorf("record %s: %w", name, err)
		}
	}
	return nil
}
