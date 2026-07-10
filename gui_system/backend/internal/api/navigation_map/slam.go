// SLAM 맵 제공: 대시보드는 SLAM occupancy grid를 토폴로지 아래 PNG
// 배경으로 렌더한다. 맵 파일은 SLAM 툴체인(map.yaml + map.pgm)이
// 생성하며 maps/ 아래에 있다. yaml의 `image:` 필드를 따르되, 지정된
// PGM이 없으면(운영자가 yaml을 다시 저장하지 않고 새 파일만 교체했을 때
// 흔함) 같은 디렉토리의 첫 .pgm으로 폴백한다.
package navigation_map

import (
	"bytes"
	"fmt"
	"image"
	"image/png"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"github.com/gin-gonic/gin"
	"gopkg.in/yaml.v3"
)

type mapYAMLFile struct {
	Image          string    `yaml:"image"`
	Resolution     float64   `yaml:"resolution"`
	Origin         []float64 `yaml:"origin"`
	Negate         int       `yaml:"negate"`
	OccupiedThresh float64   `yaml:"occupied_thresh"`
	FreeThresh     float64   `yaml:"free_thresh"`
}

// resolveMap은 map.yaml을 읽어 파싱된 메타데이터와 PGM 이미지의 해석된
// 절대 경로를 반환한다. 같은 디렉토리의 아무 .pgm으로 폴백하는 덕분에
// 수동 파일 교체로 yaml의 image 참조가 stale해져도 시스템이 동작한다.
func (h *Handlers) resolveMap() (*mapYAMLFile, string, error) {
	if h.MapYAML == "" {
		return nil, "", fmt.Errorf("map_yaml not configured")
	}
	data, err := os.ReadFile(h.MapYAML)
	if err != nil {
		return nil, "", err
	}
	var m mapYAMLFile
	if err := yaml.Unmarshal(data, &m); err != nil {
		return nil, "", err
	}
	dir := filepath.Dir(h.MapYAML)
	imgPath := filepath.Join(dir, m.Image)
	if _, err := os.Stat(imgPath); err != nil {
		entries, _ := os.ReadDir(dir)
		for _, e := range entries {
			if !e.IsDir() && strings.EqualFold(filepath.Ext(e.Name()), ".pgm") {
				imgPath = filepath.Join(dir, e.Name())
				break
			}
		}
	}
	return &m, imgPath, nil
}

// MapInfo는 처리된(정리 + 크롭 + 회전) 맵 이미지의 크기 + resolution을
// 반환한다. 렌더러는 resolution × dims로 world bbox를 도출해 초기 viewport
// 크기와 world 좌표 맵 그리기에 모두 사용한다(그래서 pan/zoom이 로봇/경로와
// 함께 맵에도 적용됨).
func (h *Handlers) MapInfo(c *gin.Context) {
	m, _, err := h.resolveMap()
	if err != nil {
		c.JSON(http.StatusOK, gin.H{"available": false})
		return
	}
	img, meta, err := h.loadProcessed()
	if err != nil {
		c.JSON(http.StatusOK, gin.H{"available": false})
		return
	}
	b := img.Bounds()
	res := m.Resolution
	if res <= 0 {
		res = 0.05
	}
	var originX, originY float64
	if len(m.Origin) >= 2 {
		originX, originY = m.Origin[0], m.Origin[1]
	}
	// width/height는 처리된(crop+회전) 이미지 크기. origin/orig_*/crop_*는
	// 프론트가 world 좌표 → 처리 이미지 픽셀 매핑을 정확히 복원해 노드·로봇·
	// 경로를 맵 위 올바른 위치에 겹치도록 하기 위한 변환 파라미터다.
	c.JSON(http.StatusOK, gin.H{
		"available":   true,
		"image_url":   "/api/v1/map/image",
		"width":       b.Dx(),
		"height":      b.Dy(),
		"resolution":  res,
		"fit":         true,
		"origin_x":    originX,
		"origin_y":    originY,
		"orig_width":  meta.OrigW,
		"orig_height": meta.OrigH,
		"crop_min_x":  meta.MinX,
		"crop_min_y":  meta.MinY,
		"crop_w":      meta.CW,
		"crop_h":      meta.CH,
	})
}

// MapImage는 처리된 PNG를 반환한다(unknown→투명, SLAM 스캔 영역으로
// 크롭, 90° 반시계 회전).
func (h *Handlers) MapImage(c *gin.Context) {
	img, _, err := h.loadProcessed()
	if err != nil {
		c.Status(http.StatusNotFound)
		return
	}
	c.Header("Cache-Control", "no-store")
	var buf bytes.Buffer
	if err := png.Encode(&buf, img); err != nil {
		c.Status(http.StatusInternalServerError)
		return
	}
	c.Data(http.StatusOK, "image/png", buf.Bytes())
}

// loadProcessed는 PGM을 읽고 고립 장애물 blob(SLAM 노이즈)을 제거한 뒤,
// 남은 벽 픽셀의 bbox로 타이트하게 크롭한다. 클라이언트가 이 이미지를
// stretch-fit해 검은 벽 외곽선이 네비게이션 패널 가장자리에 정확히 닿아
// 여백이 남지 않게 한다.
// processedMeta는 world 좌표 ↔ 처리 이미지 픽셀 매핑을 프론트가 복원하는 데
// 필요한 기하 정보다: 원본 PGM 크기, crop bbox 오프셋, crop 크기.
type processedMeta struct {
	OrigW, OrigH int // 원본 PGM 픽셀 크기
	MinX, MinY   int // crop bbox 좌상단(원본 픽셀)
	CW, CH       int // crop 영역 크기(회전 전)
}

func (h *Handlers) loadProcessed() (image.Image, processedMeta, error) {
	var meta processedMeta
	_, imgPath, err := h.resolveMap()
	if err != nil {
		return nil, meta, err
	}
	gray, w, ht, err := decodePGMGray(imgPath)
	if err != nil {
		return nil, meta, err
	}
	meta.OrigW, meta.OrigH = w, ht
	removeIslandObstacles(gray, w, ht)
	minX, minY, maxX, maxY := findDarkBBox(gray, w, ht)
	if maxX < 0 {
		return image.NewNRGBA(image.Rect(0, 0, 0, 0)), meta, nil
	}
	cw := maxX - minX + 1
	ch := maxY - minY + 1
	meta.MinX, meta.MinY, meta.CW, meta.CH = minX, minY, cw, ch
	cropped := cropToBBoxWithAlpha(gray, w, minX, minY, cw, ch)
	return rotateCCW90(cropped, cw, ch), meta, nil
}

// findDarkBBox는 raw gray 버퍼에서 모든 occupied 픽셀(v < 100)의 경계
// 사각형을 반환한다. island removal 이전에 실행해, SLAM이 분리된 벽
// 조각을 만들어도 bbox가 실제 벽 범위를 반영하게 한다.
func findDarkBBox(gray []uint8, w, h int) (int, int, int, int) {
	const occThresh = 100
	minX, minY, maxX, maxY := w, h, -1, -1
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if gray[y*w+x] < occThresh {
				if x < minX {
					minX = x
				}
				if y < minY {
					minY = y
				}
				if x > maxX {
					maxX = x
				}
				if y > maxY {
					maxY = y
				}
			}
		}
	}
	return minX, minY, maxX, maxY
}

// cropToBBoxWithAlpha는 주어진 gray 버퍼 사각형을 새 NRGBA로 복사하며,
// unknown 셀을 alpha 0으로 매핑해 배경이 비치게 한다. bbox는 호출 측이
// 제공한다(gray 버퍼의 파괴적 정리 이전에 미리 계산됨).
func cropToBBoxWithAlpha(gray []uint8, w, minX, minY, cw, ch int) *image.NRGBA {
	out := image.NewNRGBA(image.Rect(0, 0, cw, ch))
	for y := 0; y < ch; y++ {
		for x := 0; x < cw; x++ {
			v := gray[(minY+y)*w+(minX+x)]
			i := y*out.Stride + x*4
			if isUnknown(v) {
				out.Pix[i+3] = 0
			} else {
				out.Pix[i+0] = v
				out.Pix[i+1] = v
				out.Pix[i+2] = v
				out.Pix[i+3] = 255
			}
		}
	}
	return out
}

// removeIslandObstacles는 크기가 minIslandPixels 미만인 어두운(occupied)
// connected component를 지우고 그 픽셀을 free-space 값(254)으로 바꾼다.
// 방 안의 진짜 장애물은 보존하면서 SLAM 스캔 노이즈(예: 맵 가장자리의
// 1~4개 흩어진 픽셀)는 제거한다 — 가운데 작은 박스 모양 장애물은 그
// 자체로 하나의 component라, 가장 큰 component만 남기면 지워져 버린다.
func removeIslandObstacles(gray []uint8, w, h int) {
	const occThresh = 100      // 이 값 미만은 "occupied"로 취급
	const minIslandPixels = 20 // 가운데 장애물(≈78px)은 유지하고 1~4px 노이즈는 버리도록 튜닝
	occupied := func(v uint8) bool { return v < occThresh }
	visited := make([]bool, w*h)
	queue := make([]int, 0, 64)
	// 각 component의 픽셀 인덱스를 추적해 component 단위로 유지할지
	// 지울지 결정한다.
	keep := make([]bool, w*h)
	component := make([]int, 0, 64)
	for s := 0; s < w*h; s++ {
		if visited[s] || !occupied(gray[s]) {
			continue
		}
		queue = queue[:0]
		component = component[:0]
		queue = append(queue, s)
		visited[s] = true
		for len(queue) > 0 {
			cur := queue[len(queue)-1]
			queue = queue[:len(queue)-1]
			component = append(component, cur)
			cx, cy := cur%w, cur/w
			neighbours := [4]int{cur - 1, cur + 1, cur - w, cur + w}
			valid := [4]bool{cx > 0, cx < w-1, cy > 0, cy < h-1}
			for k, n := range neighbours {
				if valid[k] && !visited[n] && occupied(gray[n]) {
					visited[n] = true
					queue = append(queue, n)
				}
			}
		}
		if len(component) >= minIslandPixels {
			for _, idx := range component {
				keep[idx] = true
			}
		}
	}
	for i := 0; i < w*h; i++ {
		if occupied(gray[i]) && !keep[i] {
			gray[i] = 254
		}
	}
}

// decodePGMGray는 raw 8비트 gray 버퍼와 크기를 함께 반환한다. 하위
// 코드가 이를 한 번에 크롭 + 회전하며 NRGBA로 변환한다.
func decodePGMGray(path string) ([]uint8, int, int, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, 0, 0, err
	}
	w, ht, maxv, magic, dataStart, err := parsePGMHeader(data)
	if err != nil {
		return nil, 0, 0, err
	}
	pix := make([]uint8, w*ht)
	switch magic {
	case "P5":
		if maxv <= 255 {
			need := w * ht
			if dataStart+need > len(data) {
				return nil, 0, 0, fmt.Errorf("pgm: truncated body (need %d, have %d)", need, len(data)-dataStart)
			}
			copy(pix, data[dataStart:dataStart+need])
		} else {
			for i := 0; i < w*ht; i++ {
				pix[i] = data[dataStart+i*2]
			}
		}
	case "P2":
		pos := dataStart
		for i := 0; i < w*ht; i++ {
			tok, p := pgmReadToken(data, pos)
			pos = p
			v, _ := strconv.Atoi(tok)
			pix[i] = uint8(v)
		}
	default:
		return nil, 0, 0, fmt.Errorf("pgm: unsupported magic %q", magic)
	}
	return pix, w, ht, nil
}

// isUnknown은 ROS map_server 컨벤션을 따른다(occ_thresh가 -0.65~0.65이고
// 기본 254 free / 0 occupied일 때 unknown은 셀값 205). 약간의 작성 차이를
// 흡수하기 위해 폭을 넓게 잡는다.
func isUnknown(v uint8) bool { return v >= 200 && v <= 210 }

// rotateCCW90은 source를 90° 반시계로 회전한다. 출력 dims는 뒤바뀐다
// (h × w). 픽셀 매핑: source (x,y) → target (y, w-1-x).
func rotateCCW90(src *image.NRGBA, w, h int) *image.NRGBA {
	out := image.NewNRGBA(image.Rect(0, 0, h, w))
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			si := y*src.Stride + x*4
			tx := y
			ty := w - 1 - x
			ti := ty*out.Stride + tx*4
			copy(out.Pix[ti:ti+4], src.Pix[si:si+4])
		}
	}
	return out
}

// parsePGMHeader는 width, height, maxval, magic과 픽셀 payload가 시작되는
// 인덱스를 추출한다. PGM 헤더는 '#' 주석과 토큰 사이 임의 공백을 허용하며,
// 양쪽 모두 처리한다.
func parsePGMHeader(data []byte) (w, h, maxv int, magic string, bodyStart int, err error) {
	pos := 0
	magic, pos = pgmReadToken(data, pos)
	if magic != "P5" && magic != "P2" {
		err = fmt.Errorf("pgm: unsupported magic %q", magic)
		return
	}
	var t string
	t, pos = pgmReadToken(data, pos)
	w, err = strconv.Atoi(t)
	if err != nil {
		return
	}
	t, pos = pgmReadToken(data, pos)
	h, err = strconv.Atoi(t)
	if err != nil {
		return
	}
	t, pos = pgmReadToken(data, pos)
	maxv, err = strconv.Atoi(t)
	if err != nil {
		return
	}
	// binary PGM에서는 다음 단일 공백 바이트가 픽셀 payload의 시작을
	// 표시한다 — pgmReadToken은 토큰 뒤 첫 공백까지만 소비하므로 여기서
	// 구분자 바이트 하나를 정확히 건너뛴다.
	if pos < len(data) {
		pos++
	}
	bodyStart = pos
	return
}

// pgmReadToken은 공백 + '#' 주석을 건너뛰고 다음 공백 종료 토큰을 읽는다.
// 토큰과 그 직후 위치를 반환한다.
func pgmReadToken(data []byte, pos int) (string, int) {
	for pos < len(data) {
		b := data[pos]
		if b == '#' {
			for pos < len(data) && data[pos] != '\n' {
				pos++
			}
		} else if b == ' ' || b == '\t' || b == '\r' || b == '\n' {
			pos++
		} else {
			break
		}
	}
	start := pos
	for pos < len(data) {
		b := data[pos]
		if b == ' ' || b == '\t' || b == '\r' || b == '\n' {
			break
		}
		pos++
	}
	return string(data[start:pos]), pos
}
