"""Local HTTP replay server running inside humble_x64_env docker container.

Enables one-click 3D GUI replay from browser index.html directly into DISPLAY=:0.
"""
import json
import os
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse

PORT = 8766
current_replay_proc = None


class ReplayHandler(BaseHTTPRequestHandler):
    def _send_cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization")

    def do_OPTIONS(self):
        self.send_response(204)
        self._send_cors_headers()
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/health"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._send_cors_headers()
            self.end_headers()
            resp = {"status": "ok", "service": "tomato_replay_server", "port": PORT}
            self.wfile.write(json.dumps(resp).encode("utf-8"))
            return

        if parsed.path == "/replay":
            query = parse_qs(parsed.query)
            cmd = query.get("cmd", [""])[0]
            self.handle_command(cmd)
            return

        self.send_response(404)
        self._send_cors_headers()
        self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/replay":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length).decode("utf-8")
            try:
                data = json.loads(body) if body else {}
                cmd = data.get("command", "")
            except Exception as e:
                self.send_response(400)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self._send_cors_headers()
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "error": f"Invalid JSON: {e}"}).encode("utf-8"))
                return

            self.handle_command(cmd)
            return

        self.send_response(404)
        self._send_cors_headers()
        self.end_headers()

    def handle_command(self, cmd: str):
        global current_replay_proc
        cmd = cmd.strip()
        if not cmd:
            self.send_response(400)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._send_cors_headers()
            self.end_headers()
            self.wfile.write(json.dumps({"status": "error", "error": "Empty command"}).encode("utf-8"))
            return

        # Ensure DISPLAY is set
        env = os.environ.copy()
        if "DISPLAY" not in env or not env["DISPLAY"]:
            env["DISPLAY"] = ":0"

        # Safe launch in background
        try:
            # Terminate previous viewer if still running to avoid cluttering screens
            if current_replay_proc is not None and current_replay_proc.poll() is None:
                try:
                    current_replay_proc.terminate()
                except Exception:
                    pass

            print(f"[ReplayServer] Executing: {cmd}", flush=True)
            current_replay_proc = subprocess.Popen(
                cmd,
                shell=True,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._send_cors_headers()
            self.end_headers()
            resp = {
                "status": "ok",
                "message": "3D 리플레이가 실행되었습니다.",
                "pid": current_replay_proc.pid,
                "display": env.get("DISPLAY", ":0"),
                "command": cmd
            }
            self.wfile.write(json.dumps(resp).encode("utf-8"))

        except Exception as e:
            self.send_response(500)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._send_cors_headers()
            self.end_headers()
            self.wfile.write(json.dumps({"status": "error", "error": str(e)}).encode("utf-8"))

    def log_message(self, format, *args):
        # Concise logging
        sys.stderr.write(f"[ReplayServer] {self.address_string()} - {format % args}\n")


def main():
    server = HTTPServer(("0.0.0.0", PORT), ReplayHandler)
    print(f"[ReplayServer] Listening on http://0.0.0.0:{PORT} (CORS enabled)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[ReplayServer] Shutting down.", flush=True)
        server.server_close()


if __name__ == "__main__":
    main()

