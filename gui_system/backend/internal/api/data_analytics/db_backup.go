package data_analytics

import (
	"context"
	"log/slog"
	"os"
	"path/filepath"
	"sort"
	"time"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/database"
)

// RunBackupLoop은 하루에 한 번 DB를 스냅샷하고 `retainDays`보다 오래된
// 복사본을 정리한다. 설계 §7-5에 따라 VACUUM INTO를 사용해 진행 중인 WAL
// 페이지와 일관된 스냅샷을 만든다.
func RunBackupLoop(ctx context.Context, d *database.DB, dir string, retainDays int, logger *slog.Logger) {
	if dir == "" {
		return
	}
	t := time.NewTicker(24 * time.Hour)
	defer t.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-t.C:
			if path, err := d.Backup(ctx, dir); err != nil {
				logger.Warn("db backup failed", slog.String("err", err.Error()))
			} else {
				logger.Info("db backup written", slog.String("path", path))
			}
			pruneOldBackups(dir, retainDays, logger)
		}
	}
}

func pruneOldBackups(dir string, retainDays int, logger *slog.Logger) {
	if retainDays <= 0 {
		return
	}
	entries, err := os.ReadDir(dir)
	if err != nil {
		return
	}
	type entry struct {
		name    string
		modTime time.Time
	}
	files := make([]entry, 0, len(entries))
	for _, e := range entries {
		if e.IsDir() {
			continue
		}
		info, err := e.Info()
		if err != nil {
			continue
		}
		files = append(files, entry{name: e.Name(), modTime: info.ModTime()})
	}
	sort.Slice(files, func(i, j int) bool { return files[i].modTime.After(files[j].modTime) })
	cutoff := time.Now().Add(-time.Duration(retainDays) * 24 * time.Hour)
	for _, f := range files {
		if f.modTime.Before(cutoff) {
			path := filepath.Join(dir, f.name)
			if err := os.Remove(path); err != nil {
				logger.Warn("prune backup", slog.String("path", path), slog.String("err", err.Error()))
			}
		}
	}
}
