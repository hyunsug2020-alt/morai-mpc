#!/usr/bin/env python3
"""Local Unix-socket controls for the Scenario Studio's repeated-drive page."""
import json
import os
import signal
import socketserver
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse

from competition_cycle import STATE, TRAFFIC_STATUS


class Control:
    def __init__(self):
        self.process = None
        self.stopping = False
        self.lock = threading.Lock()

    def active(self):
        return self.process is not None and self.process.poll() is None

    def start(self):
        with self.lock:
            if self.active():
                return 409, dict(error='자동 반복 주행이 이미 실행 중임')
            env = dict(os.environ)
            self.process = subprocess.Popen([sys.executable, str(Path(__file__).with_name('competition_cycle.py'))], env=env)
            self.stopping = False
            return 200, dict(ok=True, running=True)

    def stop(self):
        with self.lock:
            if self.active():
                self.stopping = True
                self.process.terminate()
            return 200, dict(ok=True, stopping=self.active())

    def status(self):
        def read(path, default):
            try:
                return json.loads(path.read_text())
            except (OSError, ValueError):
                return default
        state = read(STATE/'status.json', dict(phase='stopped', round=0))
        active = self.active()
        if not active:
            state['phase'] = 'stopped'
        elif self.stopping:
            state['phase'] = 'stopping'
        elif state.get('phase') == 'stopped':
            state['phase'] = 'starting'
        history = []
        path = STATE/'results.jsonl'
        if path.exists():
            for line in path.read_text().splitlines():
                try:
                    history.append(json.loads(line))
                except ValueError:
                    continue
        return dict(available=True, running=active, state=state,
                    traffic=read(TRAFFIC_STATUS, {}), history=history[-15:][::-1],
                    totals=dict(runs=len(history), laps=sum(r.get('outcome') == 'lap_complete' for r in history),
                                collisions=sum(r.get('collision_events', 0) for r in history)))


class Server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def reply(self, status, value):
        data = json.dumps(value, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if urlparse(self.path).path == '/status':
            self.reply(200, self.server.control.status())
        else:
            self.reply(404, dict(error='알 수 없는 요청'))

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        if length > 8192:
            self.reply(413, dict(error='요청 크기 초과'))
            return
        if length:
            self.rfile.read(length)
        action = urlparse(self.path).path
        if action == '/start':
            self.reply(*self.server.control.start())
        elif action == '/stop':
            self.reply(*self.server.control.stop())
        else:
            self.reply(404, dict(error='알 수 없는 요청'))


def main():
    STATE.mkdir(parents=True, exist_ok=True)
    socket = STATE/'control.sock'
    if socket.exists():
        socket.unlink()
    control = Control()
    stopped = threading.Event()
    def stop(*_):
        stopped.set()
        control.stop()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    with Server(str(socket), Handler) as server:
        os.chmod(socket, 0o660)
        server.control = control
        server.timeout = .5
        try:
            while not stopped.is_set():
                server.handle_request()
        finally:
            control.stop()
            if control.active():
                try:
                    control.process.wait(75)
                except subprocess.TimeoutExpired:
                    control.process.kill()
    socket.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
