package database

import (
	"context"
	"database/sql"
	"fmt"
	"os"
	"path/filepath"
	"time"

	_ "modernc.org/sqlite"
)

// DB는 모든 리포지토리가 쓰는 단일 SQLite 커넥션을 감싼다. 설계 §7-4에
// 따라 Open 시 WAL을 활성화해, 읽기(상태 API)가 텔레메트리 배치 writer와
// 즉시 미션 쓰기를 막지 않게 한다.
type DB struct {
	conn *sql.DB
	path string
}

// Open은 완전히 초기화된 DB를 반환한다. migrations 디렉토리가 즉시
// 적용되므로 호출 측은 별도의 "migrate" 단계 없이 갓 생성된 파일을 모든
// 리포지토리 생성자에 넘길 수 있다.
func Open(path string, wal bool, migrationsDir string) (*DB, error) {
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return nil, err
	}
	conn, err := sql.Open("sqlite", path)
	if err != nil {
		return nil, fmt.Errorf("open sqlite: %w", err)
	}
	conn.SetMaxOpenConns(1) // sqlite은 단일 writer; 드라이버 레벨에서 직렬화
	if wal {
		if _, err := conn.Exec("PRAGMA journal_mode=WAL;"); err != nil {
			return nil, fmt.Errorf("set wal: %w", err)
		}
	}
	if _, err := conn.Exec("PRAGMA synchronous=NORMAL;"); err != nil {
		return nil, err
	}
	d := &DB{conn: conn, path: path}
	if migrationsDir != "" {
		if err := RunMigrations(conn, migrationsDir); err != nil {
			return nil, fmt.Errorf("migrate: %w", err)
		}
	}
	return d, nil
}

func (d *DB) Close() error  { return d.conn.Close() }
func (d *DB) Conn() *sql.DB { return d.conn }
func (d *DB) Path() string  { return d.path }

// Backup은 SQLite의 VACUUM INTO로 자기 일관성 있는 복사본을 만든다.
// 설계 §7-5에 따라, .db 파일만으로는 진행 중인 WAL 페이지가 빠져 있어
// 단순 파일 복사는 WAL과 함께 쓰면 안전하지 않다.
func (d *DB) Backup(ctx context.Context, dir string) (string, error) {
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return "", err
	}
	dst := filepath.Join(dir, fmt.Sprintf("gui-%s.db", time.Now().UTC().Format("20060102-150405")))
	if _, err := d.conn.ExecContext(ctx, "VACUUM INTO ?", dst); err != nil {
		return "", err
	}
	return dst, nil
}
