package audit

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sync"
	"time"
)

// Entry는 감사 로그에 기록되는 단일 명령 이벤트다. 설계 §2에 따라 DB
// 장애 시 인가 기록을 잃지 않도록 SQLite보다 파일 기반 JSONL을 우선한다.
type Entry struct {
	Timestamp string `json:"timestamp"`
	IP        string `json:"ip"`
	User      string `json:"user,omitempty"`
	Session   string `json:"session,omitempty"`
	Command   string `json:"command"`
	Result    string `json:"result"`
	RequestID string `json:"request_id,omitempty"`
	Detail    string `json:"detail,omitempty"`
}

// Logger는 라인 단위 mutex로 Entry 행을 JSON-Lines 파일에 기록한다.
type Logger struct {
	mu   sync.Mutex
	file *os.File
}

func NewLogger(path string) (*Logger, error) {
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return nil, fmt.Errorf("audit dir: %w", err)
	}
	f, err := os.OpenFile(path, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0o644)
	if err != nil {
		return nil, fmt.Errorf("open audit log %s: %w", path, err)
	}
	return &Logger{file: f}, nil
}

func (l *Logger) Write(e Entry) error {
	if e.Timestamp == "" {
		e.Timestamp = time.Now().UTC().Format(time.RFC3339Nano)
	}
	l.mu.Lock()
	defer l.mu.Unlock()
	return json.NewEncoder(l.file).Encode(e)
}

func (l *Logger) Close() error {
	if l.file == nil {
		return nil
	}
	return l.file.Close()
}
