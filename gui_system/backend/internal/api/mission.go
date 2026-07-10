package api

import (
	"context"
	"math"
	"net/http"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/google/uuid"

	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/audit"
	"github.com/ros2_ubuntu_ws/gui_system/backend/internal/domain"
)

// sendCommand는 모든 변경 엔드포인트가 쓰는 공통 idempotency + audit +
// 브릿지 흐름을 감싼다. X-Request-ID를 키로 한 idempotency cache 덕분에
// 네트워크 끊김 시 재시도해도 같은 명령을 두 번 실행하지 않고 안전하게
// 재생할 수 있다 — 설계 §3-9.
//
// 브릿지(ROS2) 미가동이어도 백엔드의 미션 컨텍스트(status·route·goal·ETA)는
// 추적·broadcast한다(README §7.12 "GUI 의도 추적"). 그래야 운영자가 봇 없이도
// 맵에서 미션별 경로 노드 필터가 즉시 동작하고, 봇이 켜지면 자연히 이어진다.
// 봇이 안 움직이는 사실은 응답의 bridge_offline 플래그로 알린다.
func (h *Handlers) sendCommand(c *gin.Context, kind domain.CommandKind, payload map[string]interface{}) {
	reqID := c.GetHeader("X-Request-ID")
	if reqID == "" {
		reqID = uuid.NewString()
	}
	if prev, ok := h.Idem.Lookup(reqID); ok {
		c.JSON(http.StatusOK, prev)
		return
	}
	cmd := domain.Command{RequestID: reqID, Kind: kind, Payload: payload}
	ctx, cancel := context.WithTimeout(c.Request.Context(), 5*time.Second)
	defer cancel()
	res, bridgeErr := h.Bridge.SendCommand(ctx, cmd)
	// 브릿지 결과와 무관하게 미션 컨텍스트는 갱신·broadcast한다 — route 발행이
	// 브릿지 라이프사이클에 묶이지 않게 하기 위함. 봇 라이브 연결이 없어도
	// 노드 필터·요약 카드·ETA가 즉시 동작한다.
	h.updateMissionState(ctx, kind, payload)
	if bridgeErr != nil {
		_ = h.Audit.Write(audit.Entry{IP: c.ClientIP(), Command: string(kind), Result: "bridge_offline", RequestID: reqID, Detail: bridgeErr.Error()})
		c.JSON(http.StatusOK, gin.H{
			"request_id":     reqID,
			"accepted":       true,
			"bridge_offline": true,
			"message":        "브릿지 미연결 — 의도는 추적되나 봇은 정지 상태입니다",
		})
		return
	}
	h.Idem.Store(reqID, res)
	_ = h.Audit.Write(audit.Entry{IP: c.ClientIP(), Command: string(kind), Result: "accepted", RequestID: reqID})
	c.JSON(http.StatusOK, res)
}

// updateMissionState는 수락된 명령에 따라 Cache의 미션 상태/목적지/ETA를
// 갱신하고, 연결된 모든 대시보드에 mission_state를 브로드캐스트한다. 실제
// 미션 FSM은 ROS2에 있지만, 운영자 명령(GUI 의도)을 백엔드가 추적해 요약
// 카드와 다중 클라이언트가 일관된 미션 컨텍스트를 공유하게 한다.
func (h *Handlers) updateMissionState(ctx context.Context, kind domain.CommandKind, payload map[string]interface{}) {
	switch kind {
	case domain.CmdStartMission, domain.CmdChangeGoal:
		goal := payloadString(payload, "target_node")
		route := h.routeTo(goal)
		eta := 0.0
		if len(route) >= 2 && h.Estimator != nil {
			eta = h.Estimator.Predict(ctx, route, nil, false)
		}
		// 새 주행 시작은 이전 미션을 닫고(actual=경과) 새 미션 행을 만든다.
		// 목적지 변경은 진행 중 미션을 이어가므로 새 행을 만들지 않는다.
		if kind == domain.CmdStartMission {
			h.completeActiveMission(ctx, domain.MissionCompleted)
			h.startMissionRow(ctx, goal, route, eta)
		}
		h.Cache.SetMission(domain.MissionRunning, goal, eta, route)
	case domain.CmdPause:
		h.Cache.SetMissionStatus(domain.MissionPaused)
	case domain.CmdResume:
		h.Cache.SetMissionStatus(domain.MissionRunning)
	case domain.CmdResetMission:
		h.completeActiveMission(ctx, domain.MissionCompleted)
		h.Cache.SetMission(domain.MissionPending, "", 0, nil)
	case domain.CmdEmergencyClear:
		h.Cache.SetEmergency(false)
	default:
		return
	}
	h.broadcastMission()
}

// broadcastMission은 현재 미션 컨텍스트를 mission_state envelope으로 fan-out
// 한다. payload 형태는 브릿지가 보내는 mission_state(StatusEvent.Payload에
// status)와 맞춰, 프론트가 한 경로로 처리하게 한다. 또한 route(노드 ID 리스트)를
// 좌표로 풀어 path_data로도 broadcast해, 맵의 sky-blue 점선이 토폴로지 경로를
// 그리게 한다(Nav2 미사용 환경에서 /plan publisher가 없어도 점선이 동작).
func (h *Handlers) broadcastMission() {
	if h.Hub == nil {
		return
	}
	snap := h.Cache.Snapshot()
	points := h.routePoints(snap.Route)
	h.Hub.BroadcastEnvelope("mission_state", gin.H{
		"event_type": "mission_state",
		"payload": gin.H{
			"status":            snap.MissionStatus,
			"current_goal_node": snap.CurrentGoalNode,
			"eta_seconds":       snap.ETASeconds,
			"route":             snap.Route,
		},
	})
	// route → path_data(kind=global). route가 비면 빈 points로 보내 직전 점선이
	// 사라진다(프런트 drawPaths가 길이<2를 스킵).
	h.Hub.BroadcastEnvelope("path_data", gin.H{
		"robot_id": "tb3_leader",
		"kind":     "global",
		"points":   points,
	})
}

// routePoints는 route(노드 ID 리스트)를 노드 좌표 (x,y) 리스트로 변환한다.
// 알 수 없는 ID는 건너뛴다(맵 로딩 직후 race 등). 비어 있으면 빈 슬라이스.
func (h *Handlers) routePoints(route []string) []domain.PathPoint {
	if len(route) == 0 {
		return []domain.PathPoint{}
	}
	idx := make(map[string]*domain.Node, len(h.Nodes))
	for i := range h.Nodes {
		idx[h.Nodes[i].ID] = &h.Nodes[i]
	}
	pts := make([]domain.PathPoint, 0, len(route))
	for _, id := range route {
		if n, ok := idx[id]; ok {
			pts = append(pts, domain.PathPoint{X: n.X, Y: n.Y})
		}
	}
	return pts
}

// startMissionRow는 새 미션을 DB에 기록하고 활성 미션으로 추적한다.
func (h *Handlers) startMissionRow(ctx context.Context, goal string, route []string, eta float64) {
	if h.Missions == nil || goal == "" {
		return
	}
	now := time.Now().UTC()
	start := ""
	if len(route) > 0 {
		start = route[0]
	}
	m := domain.Mission{
		ID:         uuid.NewString(),
		Type:       domain.MissionDelivery,
		Status:     domain.MissionRunning,
		TargetNode: goal,
		StartNode:  start,
		Waypoints:  route,
		StartTime:  &now,
	}
	if err := h.Missions.Create(ctx, m, eta); err != nil {
		return
	}
	h.missionMu.Lock()
	h.activeMissionID = m.ID
	h.missionStart = now
	h.missionMu.Unlock()
}

// completeActiveMission은 진행 중 미션이 있으면 actual_eta(시작 후 경과 초)와
// 함께 닫는다. 미션 이력·ETA 예측 차트가 이 행으로 채워진다.
func (h *Handlers) completeActiveMission(ctx context.Context, status domain.MissionStatus) {
	h.missionMu.Lock()
	id := h.activeMissionID
	start := h.missionStart
	h.activeMissionID = ""
	h.missionMu.Unlock()
	if id == "" || h.Missions == nil {
		return
	}
	actual := time.Since(start).Seconds()
	_ = h.Missions.Complete(ctx, id, status, actual)
}

func payloadString(p map[string]interface{}, key string) string {
	if p == nil {
		return ""
	}
	if v, ok := p[key].(string); ok {
		return v
	}
	return ""
}

// routeTo는 시작 노드 → goal 최단 경로(엣지 expected_time 기준) 노드 리스트를
// 만든다. 시작 노드는 리더 로봇 위치에 가장 가까운 노드, 없으면 loading(상차
// 구역), 그것도 없으면 첫 노드. ETA와 맵 노드 필터·라이트가 이 경로를 공유한다.
func (h *Handlers) routeTo(goal string) []string {
	if goal == "" {
		return nil
	}
	start := h.missionStartNode()
	if start == "" || start == goal {
		return []string{goal}
	}
	if r := h.shortestRoute(start, goal); len(r) >= 2 {
		return r
	}
	// 경로 탐색 실패 시에도 목적지는 보여준다.
	return []string{goal}
}

// missionStartNode는 ETA 경로의 출발 노드를 고른다.
func (h *Handlers) missionStartNode() string {
	robots := h.Cache.RobotsSnapshot()
	var leader *domain.RobotState
	for i := range robots {
		if strings.Contains(strings.ToLower(robots[i].RobotID), "leader") {
			leader = &robots[i]
			break
		}
	}
	if leader == nil && len(robots) > 0 {
		leader = &robots[0]
	}
	if leader != nil && len(h.Nodes) > 0 {
		best, bestD := "", math.MaxFloat64
		for _, n := range h.Nodes {
			d := math.Hypot(n.X-leader.Pose.X, n.Y-leader.Pose.Y)
			if d < bestD {
				bestD, best = d, n.ID
			}
		}
		if best != "" {
			return best
		}
	}
	for _, n := range h.Nodes {
		if n.ID == "loading" {
			return n.ID
		}
	}
	if len(h.Nodes) > 0 {
		return h.Nodes[0].ID
	}
	return ""
}

// shortestRoute는 토폴로지 엣지 위에서 expected_time을 비용으로 Dijkstra를
// 돌려 start→goal waypoint 리스트를 만든다. 노드 수가 작아 선형 탐색 Dijkstra로
// 충분하다. 경로 없음이면 nil.
func (h *Handlers) shortestRoute(start, goal string) []string {
	type link struct {
		to   string
		cost float64
	}
	adj := map[string][]link{}
	for _, e := range h.Edges {
		cost := e.ExpectedTime
		if cost <= 0 {
			cost = e.Distance
		}
		adj[e.FromNode] = append(adj[e.FromNode], link{e.ToNode, cost})
		if e.Direction == domain.EdgeTwoWay {
			adj[e.ToNode] = append(adj[e.ToNode], link{e.FromNode, cost})
		}
	}
	dist := map[string]float64{start: 0}
	prev := map[string]string{}
	visited := map[string]bool{}
	for {
		cur, curD := "", math.MaxFloat64
		for n, d := range dist {
			if !visited[n] && d < curD {
				cur, curD = n, d
			}
		}
		if cur == "" || cur == goal {
			break
		}
		visited[cur] = true
		for _, l := range adj[cur] {
			if nd := curD + l.cost; nd < valueOr(dist, l.to, math.MaxFloat64) {
				dist[l.to] = nd
				prev[l.to] = cur
			}
		}
	}
	if _, ok := dist[goal]; !ok {
		return nil
	}
	route := []string{goal}
	for n := goal; n != start; {
		p, ok := prev[n]
		if !ok {
			return nil
		}
		route = append([]string{p}, route...)
		n = p
	}
	return route
}

func valueOr(m map[string]float64, k string, def float64) float64 {
	if v, ok := m[k]; ok {
		return v
	}
	return def
}

// StartMission은 임의 형태의 JSON 본문(target_node, type, item_id, …)을
// 받아 Start 명령 payload로 forward한다. 모든 미션 관련 명령은 관리자
// 세션을 요구한다.
func (h *Handlers) StartMission(c *gin.Context) {
	var body map[string]interface{}
	_ = c.ShouldBindJSON(&body)
	target := payloadString(body, "target_node")
	switch target {
	case "a_leader_slot":
		h.startAreaMission(c, body, "a")
		return
	case "b_leader_slot":
		h.startAreaMission(c, body, "b")
		return
	case "charger_front", "charger_entry", "charger":
		if h.Runner != nil {
			ctx, cancel := context.WithTimeout(c.Request.Context(), 8*time.Second)
			defer cancel()
			if err := h.Runner.PrepareChargerInterrupt(ctx); err != nil {
				c.JSON(http.StatusInternalServerError, gin.H{
					"accepted": false,
					"message":  err.Error(),
				})
				return
			}
		}
	}
	h.sendCommand(c, domain.CmdStartMission, body)
}

func (h *Handlers) startAreaMission(c *gin.Context, body map[string]interface{}, area string) {
	reqID := c.GetHeader("X-Request-ID")
	if reqID == "" {
		reqID = uuid.NewString()
	}
	if prev, ok := h.Idem.Lookup(reqID); ok {
		c.JSON(http.StatusOK, prev)
		return
	}
	if h.Runner == nil {
		c.JSON(http.StatusInternalServerError, gin.H{
			"request_id": reqID,
			"accepted":   false,
			"message":    "mission runner unavailable",
		})
		return
	}

	ctx, cancel := context.WithTimeout(c.Request.Context(), 10*time.Second)
	defer cancel()
	if err := h.Runner.StartArea(ctx, area); err != nil {
		_ = h.Audit.Write(audit.Entry{
			IP:        c.ClientIP(),
			Command:   "StartAreaMission",
			Result:    "failed",
			RequestID: reqID,
			Detail:    err.Error(),
		})
		c.JSON(http.StatusInternalServerError, gin.H{
			"request_id": reqID,
			"accepted":   false,
			"message":    err.Error(),
		})
		return
	}

	h.updateMissionState(ctx, domain.CmdStartMission, body)
	res := domain.CommandResult{
		RequestID: reqID,
		Accepted:  true,
		Message:   area + " area mission started",
	}
	h.Idem.Store(reqID, res)
	_ = h.Audit.Write(audit.Entry{
		IP:        c.ClientIP(),
		Command:   "StartAreaMission",
		Result:    "accepted",
		RequestID: reqID,
		Detail:    area,
	})
	c.JSON(http.StatusOK, res)
}

func (h *Handlers) Pause(c *gin.Context)        { h.sendCommand(c, domain.CmdPause, nil) }
func (h *Handlers) Resume(c *gin.Context)       { h.sendCommand(c, domain.CmdResume, nil) }
func (h *Handlers) ResetMission(c *gin.Context) { h.sendCommand(c, domain.CmdResetMission, nil) }

func (h *Handlers) ChangeGoal(c *gin.Context) {
	var body map[string]interface{}
	_ = c.ShouldBindJSON(&body)
	h.sendCommand(c, domain.CmdChangeGoal, body)
}

// EmergencyStop은 관리자 세션 없이 동작하는 유일한 변경 엔드포인트다 —
// 대신 core/의 IP allow-list + rate limiter 미들웨어가 게이트한다. 감사
// 로깅은 미들웨어(차단됨)와 여기(수락됨) 양쪽에서 일어나 모든 호출이
// 기록을 남긴다.
func (h *Handlers) EmergencyStop(c *gin.Context) {
	reqID := uuid.NewString()
	ctx, cancel := context.WithTimeout(c.Request.Context(), 2*time.Second)
	defer cancel()
	res, bridgeErr := h.Bridge.SendCommand(ctx, domain.Command{RequestID: reqID, Kind: domain.CmdEmergencyStop})
	// 긴급 정지는 봇 라이브 연결과 무관하게 백엔드 상태에 즉시 반영한다 — 운영자가
	// 누른 의도를 GUI에 즉시 표시하고, 봇이 켜지면 다음 명령부터 반영되도록.
	h.Cache.SetEmergency(true)
	h.broadcastMission()
	if bridgeErr != nil {
		_ = h.Audit.Write(audit.Entry{IP: c.ClientIP(), Command: "EmergencyStop", Result: "bridge_offline", RequestID: reqID, Detail: bridgeErr.Error()})
		c.JSON(http.StatusOK, gin.H{"request_id": reqID, "accepted": true, "bridge_offline": true})
		return
	}
	_ = h.Audit.Write(audit.Entry{IP: c.ClientIP(), Command: "EmergencyStop", Result: "accepted", RequestID: reqID})
	c.JSON(http.StatusOK, res)
}

func (h *Handlers) EmergencyClear(c *gin.Context) { h.sendCommand(c, domain.CmdEmergencyClear, nil) }

// PoseEstimate는 RViz 스타일 2D Pose Estimate를 브릿지로 forward한다.
// payload {x, y, theta}는 그대로 전달되며 — 브릿지가
// geometry_msgs/PoseWithCovarianceStamped로 변환해 /initialpose에
// 발행한다. 관리자 인증 없음: 로봇을 이동시키는 명령이 아니라 위치추정
// 보조 기능이기 때문.
//
// 성공 시 pose는 WS로 echo되어 다른 모든 대시보드 클라이언트(PC + 폰)가
// 마커를 잠시 렌더한다.
func (h *Handlers) PoseEstimate(c *gin.Context) {
	var body map[string]interface{}
	if err := c.ShouldBindJSON(&body); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}
	h.sendCommand(c, domain.CmdSetPoseEstimate, body)
	if h.Hub != nil {
		h.Hub.BroadcastEnvelope("pose_estimate", body)
	}
}
