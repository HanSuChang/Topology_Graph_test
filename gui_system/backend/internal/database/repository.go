package database

// Repository는 모든 엔티티 리포지토리가 임베드하는 작은 공유 베이스다.
// 핸들러가 구체 *DB 타입 대신 추상 DB 핸들에 의존할 수 있게 해, 테스트에서
// 모킹을 쉽게 만든다.
type Repository struct {
	db *DB
}

func NewRepository(d *DB) Repository { return Repository{db: d} }

func (r Repository) DB() *DB { return r.db }
