"""Local IPC for the web UI; ROS publishing stays in the mapping main loop."""
import json
import os
import socketserver
import threading
import time


class MappingControl:
    def __init__(self, path):
        self.lock = threading.Lock()
        self.state = {'phase': 'starting', 'error': None, 'saved_path': None}
        self.key = ('x', 0.)
        self.save_requested = False
        self.owner = None
        self.sequences = {}
        control = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.request.settimeout(2)
                try:
                    body = json.loads(self.rfile.readline(4096))
                    result = control.request(body)
                except (ValueError, OSError) as error:
                    result = {'error': str(error)}
                self.wfile.write((json.dumps(result) + '\n').encode())

        self.server = socketserver.ThreadingUnixStreamServer(str(path), Handler)
        self.server.daemon_threads = True
        os.chmod(path, 0o600)
        self.path = path
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def request(self, body):
        with self.lock:
            if not isinstance(body, dict):
                raise ValueError('请求必须为 JSON 对象')
            action = body.get('action')
            if action == 'status':
                return dict(self.state)
            if self.state['phase'] != 'ready':
                raise ValueError('建图尚未就绪，或正在保存/退出')
            if action == 'key':
                key = body.get('key')
                if key not in ('w', 's', 'a', 'd', 'q', 'e', 'x'):
                    raise ValueError('无效的移动命令')
                now = time.monotonic()
                owner = body.get('client')
                if not isinstance(owner, str) or not 1 <= len(owner) <= 100:
                    raise ValueError('缺少控制会话')
                sequence = body.get('seq')
                if not isinstance(sequence, int) or sequence < 0:
                    raise ValueError('缺少命令序号')
                if sequence <= self.sequences.get(owner, -1):
                    return {'ok': True, 'ignored': True}
                if len(self.sequences) >= 256 and owner not in self.sequences:
                    del self.sequences[next(iter(self.sequences))]
                self.sequences[owner] = sequence
                if key != 'x' and self.owner != owner and now < self.key[1] + .5:
                    raise ValueError('另一浏览器正在控制，请稍后重试')
                self.owner = owner
                self.key = (key, now if key != 'x' else 0.)
            elif action == 'save':
                self.key = ('x', 0.)
                self.state.update(phase='saving', error=None)
                self.save_requested = True
            else:
                raise ValueError('未知建图命令')
            return {'ok': True}

    def update(self, **values):
        with self.lock:
            self.state.update(values)
            if self.state['phase'] != 'ready':
                self.key = ('x', 0.)

    def take(self):
        with self.lock:
            save = self.save_requested
            self.save_requested = False
            return self.key, save

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.path.unlink(missing_ok=True)
