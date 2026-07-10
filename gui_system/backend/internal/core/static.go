package core

import (
	"net/http"
	"os"
	"path/filepath"
	"strings"

	"github.com/gin-gonic/gin"
)

// ServeStatic은 React 프로덕션 빌드를 Gin 엔진 아래에 마운트한다. PWA
// 산출물(manifest, service worker, icons)은 명시적 라우트가 필요하다.
// 그렇지 않으면 SPA 폴백이 알 수 없는 경로에 index.html을 반환해 manifest
// 아이콘 로드를 조용히 깨뜨리기 때문이다.
//
// NoRoute 핸들러는 여전히 client-side 라우트(확장자가 없고 /api·/ws 밖의
// 경로)에 대해 index.html로 폴백한다. 파일처럼 보이지만 존재하지 않는
// 경로는 진짜 404를 반환한다.
func ServeStatic(engine *gin.Engine, staticDir string) {
	if staticDir == "" {
		return
	}
	engine.Static("/assets", staticDir+"/assets")
	engine.Static("/icons", staticDir+"/icons")
	engine.StaticFile("/", staticDir+"/index.html")
	engine.StaticFile("/sw.js", staticDir+"/sw.js")
	engine.StaticFile("/registerSW.js", staticDir+"/registerSW.js")
	// manifest.webmanifest는 응답이 `application/manifest+json`을 갖도록
	// 명시적 핸들러가 필요하다 — 많은 플랫폼이 "홈 화면에 추가" 프롬프트
	// 표시 여부를 정할 때 `text/plain` manifest를 무시한다.
	engine.GET("/manifest.webmanifest", func(c *gin.Context) {
		c.Header("Content-Type", "application/manifest+json; charset=utf-8")
		c.File(staticDir + "/manifest.webmanifest")
	})
	engine.StaticFile("/manifest.json", staticDir+"/manifest.json")

	engine.NoRoute(func(c *gin.Context) {
		p := c.Request.URL.Path
		// API/WS는 여기 오면 안 되지만 방어적으로 처리한다.
		if strings.HasPrefix(p, "/api") || strings.HasPrefix(p, "/ws") {
			c.Status(http.StatusNotFound)
			return
		}
		// Workbox는 sw.js 옆에 /workbox-abcdef.js 같은 해시 형제 파일을
		// 만든다. SPA index로 폴백하기 전에 dist 루트의 *.js / *.css /
		// 이미지 요청을 먼저 제공한다.
		if ext := filepath.Ext(p); ext != "" {
			candidate := filepath.Clean(staticDir + p)
			if info, err := os.Stat(candidate); err == nil && !info.IsDir() {
				c.File(candidate)
				return
			}
			c.Status(http.StatusNotFound)
			return
		}
		c.File(staticDir + "/index.html")
	})
}
