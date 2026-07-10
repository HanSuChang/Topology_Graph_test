package missionrunner

import (
	"bufio"
	"context"
	"fmt"
	"log/slog"
	"os/exec"
	"sync"
	"syscall"
	"time"
)

type Event struct {
	Level   string
	Source  string
	Message string
}

type Logger func(Event)

type Manager struct {
	mu     sync.Mutex
	log    *slog.Logger
	emit   Logger
	procs  map[string]*exec.Cmd
	active string
}

func New(log *slog.Logger, emit Logger) *Manager {
	if log == nil {
		log = slog.Default()
	}
	return &Manager{
		log:   log,
		emit:  emit,
		procs: make(map[string]*exec.Cmd),
	}
}

func (m *Manager) StartArea(ctx context.Context, area string) error {
	profile, err := areaProfile(area)
	if err != nil {
		return err
	}

	m.mu.Lock()
	defer m.mu.Unlock()

	m.info("mission", fmt.Sprintf("%s 구역 자동 실행 준비", profile.Label))
	m.stopLocked(ctx)

	if err := m.runStopCommand(ctx, "stop remote aruco", turtlebotStopArucoCommand()); err != nil {
		m.warn("mission", fmt.Sprintf("기존 aruco 정리 실패: %v", err))
	}
	if err := m.runStopCommand(ctx, "stop remote rc_follow", rcCarStopCommand()); err != nil {
		m.warn("mission", fmt.Sprintf("기존 rc_follow 정리 실패: %v", err))
	}

	aruco, err := m.startProcessLocked("aruco", turtlebotArucoCommand())
	if err != nil {
		m.err("mission", fmt.Sprintf("aruco 실행 실패: %v", err))
		return err
	}
	m.procs["aruco"] = aruco
	m.info("mission", "터틀봇 aruco detector 실행됨")

	time.Sleep(1200 * time.Millisecond)

	local, err := m.startProcessLocked(profile.LocalName, profile.LocalCommand)
	if err != nil {
		m.err("mission", fmt.Sprintf("%s 미션 실행 실패: %v", profile.Label, err))
		m.stopLocked(ctx)
		return err
	}
	m.procs["local_mission"] = local
	m.info("mission", fmt.Sprintf("%s 주행 노드 실행됨", profile.Label))

	time.Sleep(500 * time.Millisecond)

	rc, err := m.startProcessLocked("rc_follow", profile.RCCommand)
	if err != nil {
		m.err("mission", fmt.Sprintf("RC카 follow 실행 실패: %v", err))
		m.stopLocked(ctx)
		return err
	}
	m.procs["rc_follow"] = rc
	m.active = profile.ID
	m.info("mission", fmt.Sprintf("%s 구역 터틀봇/RC카 주행 시작", profile.Label))
	return nil
}

func (m *Manager) PrepareChargerInterrupt(ctx context.Context) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	m.info("mission", "충전소 interrupt 준비: A/B 미션과 RC카 정지")
	m.stopLocked(ctx)
	if err := m.runStopCommand(ctx, "stop remote rc_follow", rcCarStopCommand()); err != nil {
		m.warn("mission", fmt.Sprintf("RC카 정리 실패: %v", err))
	}

	cmd, err := m.startProcessLocked("charger_only", chargerOnlyCommand())
	if err != nil {
		m.err("mission", fmt.Sprintf("charger_only mission_loop 실행 실패: %v", err))
		return err
	}
	m.procs["charger_only"] = cmd
	m.active = "charger"
	m.info("mission", "charger_only mission_loop 실행됨")
	time.Sleep(900 * time.Millisecond)
	return nil
}

func (m *Manager) Stop(ctx context.Context) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.stopLocked(ctx)
}

func (m *Manager) stopLocked(ctx context.Context) {
	for name, cmd := range m.procs {
		if cmd.Process != nil {
			_ = syscall.Kill(-cmd.Process.Pid, syscall.SIGTERM)
			m.info("mission", fmt.Sprintf("%s 프로세스 종료 요청", name))
		}
		delete(m.procs, name)
	}
	m.active = ""

	stopCommands := []struct {
		name string
		cmd  string
	}{
		{"local missions", localStopCommand()},
		{"remote rc_follow", rcCarStopCommand()},
	}
	for _, item := range stopCommands {
		if err := m.runStopCommand(ctx, item.name, item.cmd); err != nil {
			m.warn("mission", fmt.Sprintf("%s 정리 실패: %v", item.name, err))
		}
	}
}

func (m *Manager) startProcessLocked(name string, script string) (*exec.Cmd, error) {
	cmd := exec.Command("bash", "-lc", script)
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}

	stdout, err := cmd.StdoutPipe()
	if err != nil {
		return nil, err
	}
	stderr, err := cmd.StderrPipe()
	if err != nil {
		return nil, err
	}
	if err := cmd.Start(); err != nil {
		return nil, err
	}

	go m.scanOutput(name, stdout)
	go m.scanOutput(name, stderr)
	go func() {
		err := cmd.Wait()
		m.mu.Lock()
		defer m.mu.Unlock()
		for key, proc := range m.procs {
			if proc == cmd {
				delete(m.procs, key)
			}
		}
		if err != nil {
			m.warn("mission", fmt.Sprintf("%s 종료됨: %v", name, err))
		} else {
			m.info("mission", fmt.Sprintf("%s 정상 종료", name))
		}
	}()
	return cmd, nil
}

func (m *Manager) scanOutput(name string, r interface{ Read([]byte) (int, error) }) {
	scanner := bufio.NewScanner(r)
	scanner.Buffer(make([]byte, 4096), 1024*1024)
	for scanner.Scan() {
		line := scanner.Text()
		if line == "" {
			continue
		}
		m.log.Info("mission process", slog.String("process", name), slog.String("line", line))
	}
}

func (m *Manager) runStopCommand(ctx context.Context, name string, script string) error {
	runCtx, cancel := context.WithTimeout(ctx, 4*time.Second)
	defer cancel()
	cmd := exec.CommandContext(runCtx, "bash", "-lc", script)
	if out, err := cmd.CombinedOutput(); err != nil {
		return fmt.Errorf("%s: %w (%s)", name, err, string(out))
	}
	return nil
}

func (m *Manager) info(source, msg string) { m.publish("INFO", source, msg) }
func (m *Manager) warn(source, msg string) { m.publish("WARN", source, msg) }
func (m *Manager) err(source, msg string)  { m.publish("ERROR", source, msg) }

func (m *Manager) publish(level, source, msg string) {
	m.log.Info("mission runner", slog.String("level", level), slog.String("source", source), slog.String("message", msg))
	if m.emit != nil {
		m.emit(Event{Level: level, Source: source, Message: msg})
	}
}

type profile struct {
	ID           string
	Label        string
	LocalName    string
	LocalCommand string
	RCCommand    string
}

func areaProfile(area string) (profile, error) {
	switch area {
	case "a":
		return profile{
			ID:           "a",
			Label:        "A",
			LocalName:    "turtle_mission",
			LocalCommand: localMissionCommand("ros2 run amr_topology turtle_mission"),
			RCCommand:    rcCarCommand("rc_follow"),
		}, nil
	case "b":
		return profile{
			ID:           "b",
			Label:        "B",
			LocalName:    "mission2_loop",
			LocalCommand: localMissionCommand("ros2 run amr_topology mission2_loop"),
			RCCommand:    rcCarCommand("rc_follow profile:=b"),
		}, nil
	default:
		return profile{}, fmt.Errorf("unknown area %q", area)
	}
}

func localMissionCommand(run string) string {
	return "cd /home/lee/Downloads/Topology_Graph_test-main && " +
		"source /opt/ros/humble/setup.bash && " +
		"source install/setup.bash && " +
		"export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-27} && " + run
}

func chargerOnlyCommand() string {
	return localMissionCommand("ros2 run amr_topology mission_loop --ros-args -p start_mode:=charger_only")
}

func turtlebotArucoCommand() string {
	return "ssh han2@han2.local 'cd ~/Downloads/Topology_Graph_test-main && " +
		"source /opt/ros/humble/setup.bash 2>/dev/null || true; " +
		"source ~/Downloads/Topology_Graph_test-main/install/setup.bash 2>/dev/null || source ~/Topology_Graph_test-main/install/setup.bash; " +
		"export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-27}; " +
		"ros2 launch amr_topology aruco_camera_detector.launch.py'"
}

func rcCarCommand(run string) string {
	return "ssh han@han.local 'bash -lc '\\''source ~/rc_follow.sh; export ROS_DOMAIN_ID=27; " + run + "'\\'''"
}

func localStopCommand() string {
	return "pkill -TERM -f 'ros2 run amr_topology turtle_mission' || true; " +
		"pkill -TERM -f 'ros2 run amr_topology mission2_loop' || true; " +
		"pkill -TERM -f 'ros2 run amr_topology mission_loop.*start_mode:=charger_only' || true"
}

func turtlebotStopArucoCommand() string {
	return "ssh -o ConnectTimeout=2 han2@han2.local \"pkill -TERM -f 'aruco_camera_detector.launch.py' || true\""
}

func rcCarStopCommand() string {
	return "ssh -o ConnectTimeout=2 han@han.local \"pkill -TERM -f 'rc_follow' || true\""
}
