package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"time"
)

var (
	port     = flag.Int("port", 8088, "HTTP 服务监听端口")
	distPath = flag.String("dist", "../web", "静态前端网页目录")
)

type CheckRequest struct {
	Code  string `json:"code"`
	Track string `json:"track"`
}

func main() {
	flag.Parse()

	mux := http.NewServeMux()

	// 1. API 路由
	mux.HandleFunc("/api/health", handleHealth)
	mux.HandleFunc("/api/simulate", handleSimulate)
	mux.HandleFunc("/api/tracks", handleTracks)

	// 2. 静态页面托管
	absDist, _ := filepath.Abs(*distPath)
	if _, err := os.Stat(absDist); err == nil {
		fileServer := http.FileServer(http.Dir(absDist))
		mux.Handle("/", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			if strings.HasPrefix(r.URL.Path, "/api/") {
				return
			}
			// CORS 支持
			w.Header().Set("Access-Control-Allow-Origin", "*")
			fileServer.ServeHTTP(w, r)
		}))
		log.Printf("🏎️  前端静态资源已挂载: %s", absDist)
	}

	addr := fmt.Sprintf(":%d", *port)
	log.Printf("🏁 STM32 智能车仿真服务启动于 http://localhost:%d", *port)
	if err := http.ListenAndServe(addr, corsMiddleware(mux)); err != nil {
		log.Fatalf("服务启动失败: %v", err)
	}
}

func corsMiddleware(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Access-Control-Allow-Origin", "*")
		w.Header().Set("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
		w.Header().Set("Access-Control-Allow-Headers", "Content-Type")
		if r.Method == http.MethodOptions {
			w.WriteHeader(http.StatusOK)
			return
		}
		next.ServeHTTP(w, r)
	})
}

func handleHealth(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":  "ok",
		"service": "stm32-smartcar-sim",
		"version": "0.1.0",
		"time":    time.Now().Format(time.RFC3339),
	})
}

func handleTracks(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	tracksDir := "../simcore/tracks"
	entries, err := os.ReadDir(tracksDir)
	if err != nil {
		http.Error(w, fmt.Sprintf("读取赛道失败: %v", err), http.StatusInternalServerError)
		return
	}

	var trackList []map[string]interface{}
	for _, entry := range entries {
		if strings.HasSuffix(entry.Name(), ".json") {
			content, err := os.ReadFile(filepath.Join(tracksDir, entry.Name()))
			if err == nil {
				var t map[string]interface{}
				if json.Unmarshal(content, &t) == nil {
					trackList = append(trackList, t)
				}
			}
		}
	}
	json.NewEncoder(w).Encode(trackList)
}

func handleSimulate(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "仅支持 POST", http.StatusMethodNotAllowed)
		return
	}

	body, err := io.ReadAll(io.LimitReader(r.Body, 1024*1024))
	if err != nil {
		http.Error(w, "读取请求失败", http.StatusBadRequest)
		return
	}

	var req CheckRequest
	if err := json.Unmarshal(body, &req); err != nil {
		http.Error(w, "JSON 格式错误", http.StatusBadRequest)
		return
	}

	if strings.TrimSpace(req.Code) == "" {
		http.Error(w, "代码不能为空", http.StatusBadRequest)
		return
	}

	if req.Track == "" {
		req.Track = "track01"
	}

	// 创建临时文件写入学生代码
	tmpDir, err := os.MkdirTemp("", "car-sim-*")
	if err != nil {
		http.Error(w, "创建临时目录失败", http.StatusInternalServerError)
		return
	}
	defer os.RemoveAll(tmpDir)

	tmpFile := filepath.Join(tmpDir, "student_code.c")
	if err := os.WriteFile(tmpFile, []byte(req.Code), 0644); err != nil {
		http.Error(w, "写入临时文件失败", http.StatusInternalServerError)
		return
	}

	// 优先调用本地编译出的 car-sim CLI
	reportFile := filepath.Join(tmpDir, "report.json")
	cliPaths := []string{
		"../target/release/car-sim",
		"../target/release/car-sim.exe",
		"../target/debug/car-sim",
		"../target/debug/car-sim.exe",
		"car-sim",
	}

	var cliBin string
	for _, p := range cliPaths {
		if _, err := os.Stat(p); err == nil {
			cliBin = p
			break
		}
	}

	w.Header().Set("Content-Type", "application/json")

	if cliBin != "" {
		cmd := exec.Command(cliBin, "run", tmpFile, "--track", req.Track, "--out", reportFile, "--json")
		output, err := cmd.CombinedOutput()
		if err == nil {
			if data, readErr := os.ReadFile(reportFile); readErr == nil {
				w.Write(data)
				return
			}
			w.Write(output)
			return
		}
	}

	// 若尚未编译 CLI，返回标准模拟结构供前端 fallback
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":  "client_sim_required",
		"message": "CLI 在此云端尚未编译链接，前端网页自带高精度 WebAssembly/JS 双重物理引擎引擎将直接在浏览器执行完整仿真。",
	})
}
