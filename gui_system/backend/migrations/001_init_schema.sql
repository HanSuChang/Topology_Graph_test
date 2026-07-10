-- Schema v1. Migrations are applied in db.go in order; if you add a
-- migration, append it to the migrations slice rather than editing
-- prior ones so deployed databases pick up the diff incrementally.

CREATE TABLE IF NOT EXISTS missions (
    id            TEXT PRIMARY KEY,
    type          TEXT NOT NULL,
    item_id       TEXT,
    status        TEXT NOT NULL,
    target_node   TEXT,
    start_node    TEXT,
    waypoints     TEXT,       -- JSON array
    start_time    DATETIME,
    end_time      DATETIME,
    predicted_eta REAL,
    actual_eta    REAL
);

CREATE INDEX IF NOT EXISTS idx_missions_start ON missions(start_time DESC);

CREATE TABLE IF NOT EXISTS mission_events (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    mission_id  TEXT NOT NULL,
    timestamp   DATETIME NOT NULL,
    event_type  TEXT NOT NULL,
    detail      TEXT
);

CREATE TABLE IF NOT EXISTS telemetry (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    mission_id  TEXT,
    robot_id    TEXT NOT NULL,
    timestamp   DATETIME NOT NULL,
    pose_x      REAL,
    pose_y      REAL,
    velocity    REAL,
    battery     REAL
);
CREATE INDEX IF NOT EXISTS idx_telemetry_mission ON telemetry(mission_id);

CREATE TABLE IF NOT EXISTS node_travel_times (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    from_node     TEXT NOT NULL,
    to_node       TEXT NOT NULL,
    seconds       REAL NOT NULL,
    swarm_mode    INTEGER NOT NULL DEFAULT 0,
    recorded_at   DATETIME NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_travel_pair ON node_travel_times(from_node, to_node);
