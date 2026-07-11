# 1132_bot 香橙派运行包

真实硬件启动：

```bash
cd /home/orangepi/1132_bot
./scripts/start_web_console.sh
```

只看网页/仿真模式：

```bash
cd /home/orangepi/1132_bot
./scripts/start_web_console.sh --simulate
```

浏览器访问：

```text
http://香橙派IP:8000
```

首次部署必须先准备前端生产文件。构建机需要 Node.js `20.19+`：

```bash
cd web_frontend
npm ci
npm run build
cd ..
```

健康和诊断接口：

```bash
curl http://127.0.0.1:8000/health
curl http://127.0.0.1:8000/api/diagnostics
```

详细说明见：

```text
docs/USAGE.md
```
