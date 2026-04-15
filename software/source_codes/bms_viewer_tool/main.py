import argparse, threading, socket, time, queue, sys, signal, json
from http.server import BaseHTTPRequestHandler, HTTPServer
from collections import deque
import asyncio, serial, serial.tools.list_ports, websockets
import re
import warnings
from urllib.parse import urlparse, parse_qs
from pathlib import Path

VERSION = "v36"
BUILD_STAMP = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

# Silence benign SyntaxWarnings from backslashes inside embedded JS (if any remain)
try:
  warnings.filterwarnings("ignore", category=SyntaxWarning)
except Exception:
  pass

def get_laptop_ip():
  try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]; s.close(); return ip
  except Exception: return "127.0.0.1"

def make_http_handler(args, state: dict, switch_serial_fn):
  ws_port_str = str(args.ws).encode("utf-8")
  ver_str = VERSION.encode("utf-8")

  def bind_tmpl(content, is_viewer=False):
    c = content.replace(b"__WS_PORT__", ws_port_str).replace(b"__VERSION__", ver_str)
    if is_viewer:
      style = b"style='display:none'" if args.read_only else b""
      c = c.replace(b"__SENDBAR_STYLE__", style)
    return c

  class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
      nocache = "no-cache, max-age=0, must-revalidate"
      try:
        url = urlparse(self.path)
        if url.path == "/api/serial/ports":
          ports = []
          try:
            for p in serial.tools.list_ports.comports():
              ports.append({"device": p.device, "description": getattr(p, 'description', '')})
          except Exception:
            ports = []
          body = json.dumps({"ports": ports}).encode('utf-8')
          self.send_response(200)
          self.send_header("Content-Type", "application/json; charset=utf-8")
          self.send_header("Cache-Control", nocache)
          self.send_header("Content-Length", str(len(body)))
          self.end_headers(); self.wfile.write(body); return
        if url.path == "/api/serial/status":
          s = state.get("serial")
          info = {"port": getattr(s, 'port', None), "baud": getattr(s, 'baudrate', None), "is_open": bool(getattr(s, 'is_open', False))}
          body = json.dumps(info).encode('utf-8')
          self.send_response(200)
          self.send_header("Content-Type", "application/json; charset=utf-8")
          self.send_header("Cache-Control", nocache)
          self.send_header("Content-Length", str(len(body)))
          self.end_headers(); self.wfile.write(body); return
        if url.path == "/api/serial/select":
          qs = parse_qs(url.query)
          port = (qs.get('port') or [None])[0]
          baud_s = (qs.get('baud') or [None])[0]
          baud = None
          try:
            if baud_s is not None: baud = int(baud_s)
          except Exception:
            baud = None
          ok, msg, cur = switch_serial_fn(port, baud)
          body = json.dumps({"ok": ok, "message": msg, "current": cur}).encode('utf-8')
          self.send_response(200 if ok else 400)
          self.send_header("Content-Type", "application/json; charset=utf-8")
          self.send_header("Cache-Control", nocache)
          self.send_header("Content-Length", str(len(body)))
          self.end_headers(); self.wfile.write(body); return
      except Exception:
        pass
      if self.path == "/favicon.ico":
        self.send_response(204)
        self.end_headers(); return
      if self.path in ("/", "/index.html"):
        try:
          body = bind_tmpl(load_template("index.html"), True)
        except Exception:
          body = b"Template load error"
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", nocache)
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers(); self.wfile.write(body); return
      if self.path == "/dash":
        try:
          body = bind_tmpl(load_template("dashboard.html"))
        except Exception:
          body = b"Template load error"
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", nocache)
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers(); self.wfile.write(body); return
      if self.path == "/sd":
        try:
          body = bind_tmpl(load_template("sd_analyzer.html"))
        except Exception:
          body = b"Template load error"
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", nocache)
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers(); self.wfile.write(body); return
      body = b"Not Found"
      self.send_response(404)
      self.send_header("Content-Type", "text/plain; charset=utf-8")
      self.send_header("Cache-Control", nocache)
      self.send_header("Pragma", "no-cache")
      self.send_header("Expires", "0")
      self.send_header("Content-Length", str(len(body)))
      self.end_headers(); self.wfile.write(body)
    def do_POST(self):
      nocache = "no-cache, max-age=0, must-revalidate"
      try:
        url = urlparse(self.path)
        if url.path == "/api/serial/select":
          length = int(self.headers.get('Content-Length') or '0')
          if length:
            try: _ = self.rfile.read(length)
            except Exception: pass
          qs = parse_qs(url.query)
          port = (qs.get('port') or [None])[0]
          baud_s = (qs.get('baud') or [None])[0]
          baud = None
          try:
            if baud_s is not None: baud = int(baud_s)
          except Exception:
            baud = None
          ok, msg, cur = switch_serial_fn(port, baud)
          body = json.dumps({"ok": ok, "message": msg, "current": cur}).encode('utf-8')
          self.send_response(200 if ok else 400)
          self.send_header("Content-Type", "application/json; charset=utf-8")
          self.send_header("Cache-Control", nocache)
          self.send_header("Content-Length", str(len(body)))
          self.end_headers(); self.wfile.write(body); return
      except Exception:
        pass
      self.send_response(404)
      self.end_headers()
    def log_message(self, *args, **kwargs): pass
  return Handler

def serial_reader(ser, out_q: "queue.Queue[str]", stop_evt: threading.Event):
  banner = f"\n[Serial opened: {ser.port} @ {ser.baudrate}]\n"; out_q.put(banner)
  line_buffer = ""
  while not stop_evt.is_set():
    try:
      n = ser.in_waiting; data = ser.read(n if n else 1)
      if data:
        try: text = data.decode("utf-8", errors="replace")
        except Exception: text = data.decode("latin-1", errors="replace")
        line_buffer += text
        # Send complete lines, keep incomplete line in buffer
        while "\n" in line_buffer:
          line, line_buffer = line_buffer.split("\n", 1)
          out_q.put(line + "\n")
        # Also flush buffer if it gets too large (for long lines without newlines)
        if len(line_buffer) > 1024:
          out_q.put(line_buffer)
          line_buffer = ""
      else:
        # Flush any pending data when idle
        if line_buffer:
          out_q.put(line_buffer)
          line_buffer = ""
        time.sleep(0.005)
    except serial.SerialException:
      out_q.put("\n[Serial error - reconnecting in 2s]\n"); time.sleep(2)
    except Exception: time.sleep(0.01)

def backlog_add(state, chunk: str):
  if not isinstance(chunk, str):
    try: chunk = chunk.decode("utf-8", errors="replace")
    except Exception: chunk = str(chunk)
  state["backlog"].append(chunk); state["backlog_size"] += len(chunk)
  while state["backlog_size"] > state["backlog_limit"] and state["backlog"]:
    removed = state["backlog"].popleft(); state["backlog_size"] -= len(removed)

def backlog_dump(state) -> str: return "".join(state["backlog"])

async def fanout_task(state):
  loop = asyncio.get_running_loop(); q: queue.Queue = state["out_q"]
  while True:
    chunk = await loop.run_in_executor(None, q.get)
    if not chunk:
      continue
    backlog_add(state, chunk)
    with state["clients_lock"]:
      clients = list(state["clients"])
    if not clients:
      continue
    tasks = [(c, asyncio.create_task(c.send(chunk))) for c in clients]
    done, pending = await asyncio.wait([t for _, t in tasks], timeout=0.75)
    dead = []
    for (c, t) in tasks:
      if t in pending:
        t.cancel()
        dead.append(c)
      elif t in done and t.exception() is not None:
        dead.append(c)
    if dead:
      with state["clients_lock"]:
        for d in dead:
          try: state["clients"].discard(d)
          except Exception: pass

def list_ports_hint():
  ports = [p.device for p in serial.tools.list_ports.comports()]
  return f"Available serial ports: {', '.join(ports)}" if ports else "No serial ports found."

def load_template(name):
    base = Path(__file__).parent / "templates"
    with open(base / name, "rb") as f:
        return f.read()

def main():
  ap = argparse.ArgumentParser(description="Stream USB serial to phone browser on same Wi-Fi.")
  ap.add_argument("--serial","--port",dest="port",required=False,help="Serial port"); ap.add_argument("--baud", type=int, default=250000, help="Baud rate (default 250000)")
  ap.add_argument("--http", type=int, default=8000, help="HTTP port (default 8000)"); ap.add_argument("--ws", type=int, default=8765, help="WebSocket port (default 8765)")
  ap.add_argument("--host", default="0.0.0.0", help="Bind host (default 0.0.0.0)"); ap.add_argument("--read-only", action="store_true", help="Disable sending commands from browser")
  ap.add_argument("--backlog-chars", type=int, default=800_000, help="Replay history size (chars)")
  args = ap.parse_args()

  ser = None
  if args.port:
    try:
      ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
      print(f"[!] Failed to open serial {args.port} @ {args.baud}: {e}")
      print(list_ports_hint())

  out_q: "queue.Queue[str]" = queue.Queue()
  reader_stop_evt = None
  reader_thread = None
  if ser is not None:
    reader_stop_evt = threading.Event()
    reader_thread = threading.Thread(target=serial_reader, args=(ser,out_q,reader_stop_evt), daemon=True)
    reader_thread.start()

  state = {"serial":ser, "out_q":out_q, "clients":set(), "clients_lock":threading.Lock(), "read_only":bool(args.read_only),
           "backlog":deque(), "backlog_size":0, "backlog_limit":int(getattr(args,"backlog_chars",800_000)),
           "reader_stop": reader_stop_evt, "reader_thread": reader_thread}

  def switch_serial(port, baud):
    try:
      if not port: return False, "Missing 'port'", {"port": getattr(state.get("serial"), 'port', None), "baud": getattr(state.get("serial"), 'baudrate', None), "is_open": bool(getattr(state.get("serial"), 'is_open', False))}
      if baud is None: baud = int(getattr(args, 'baud', 250000) or 250000)
    except Exception: baud = 250000
    try: new_ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e: return False, f"Failed to open {port} @ {baud}: {e}", {"port": getattr(state.get("serial"), 'port', None), "baud": getattr(state.get("serial"), 'baudrate', None), "is_open": bool(getattr(state.get("serial"), 'is_open', False))}
    try:
      if state.get("reader_stop"): 
        try: state["reader_stop"].set()
        except Exception: pass
      if state.get("reader_thread"):
        try: state["reader_thread"].join(timeout=0.3)
        except Exception: pass
    except Exception: pass
    try:
      old = state.get("serial")
      if old and getattr(old, 'is_open', False):
        try: old.close()
        except Exception: pass
    except Exception: pass
    state["serial"] = new_ser
    stop_evt = threading.Event()
    th = threading.Thread(target=serial_reader, args=(new_ser, out_q, stop_evt), daemon=True)
    th.start()
    state["reader_stop"] = stop_evt; state["reader_thread"] = th
    try: out_q.put(f"\n[Serial switched to: {new_ser.port} @ {new_ser.baudrate}]\n")
    except Exception: pass
    return True, "ok", {"port": new_ser.port, "baud": new_ser.baudrate, "is_open": bool(getattr(new_ser, 'is_open', False))}

  httpd = HTTPServer((args.host, args.http), make_http_handler(args, state, switch_serial))
  threading.Thread(target=httpd.serve_forever, daemon=True).start()

  ip = get_laptop_ip()
  print(f"\n=== BMS Viewer Tool (Modular, {VERSION}) ===")
  print("Build:", BUILD_STAMP)
  print(f"Logs page:            http://{ip}:{args.http}/")
  print(f"Dashboard page:       http://{ip}:{args.http}/dash")
  print(f"SD Analyzer:          http://{ip}:{args.http}/sd")
  print(f"WebSocket endpoint:   ws://{ip}:{args.ws}")
  if state.get("serial") and getattr(state["serial"], 'is_open', False):
    print(f"Serial:               {state['serial'].port} @ {state['serial'].baudrate}")
  else:
    print("Serial:               (not connected) - choose a port in the UI")
  print("Press Ctrl+C to stop.\n")

  def handle_sigint(signum, frame): raise KeyboardInterrupt()
  try: signal.signal(signal.SIGINT, handle_sigint)
  except Exception: pass

  async def run_async():
    async def ws_handler(ws):
      added = False
      try:
        initial = backlog_dump(state)
        if initial:
          try:
            chunk_sz = 32_768
            for i in range(0, len(initial), chunk_sz): await ws.send(initial[i:i+chunk_sz])
          except Exception: pass
        try: await ws.send("-- Connected to BMS viewer --\\n")
        except Exception: pass
        with state["clients_lock"]: state["clients"].add(ws); added = True
        if state["read_only"]: await asyncio.Future()
        else:
          async for msg in ws:
            try:
              s = state["serial"]
              if s and s.is_open: s.write(msg.encode("utf-8", errors="replace"))
            except Exception: pass
      finally:
        if added:
          with state["clients_lock"]: state["clients"].discard(ws)
    fan = asyncio.create_task(fanout_task(state))
    async with websockets.serve(ws_handler, args.host, args.ws, max_size=None, ping_interval=20, ping_timeout=20):
      try:
        while True: await asyncio.sleep(1)
      except asyncio.CancelledError: pass
      finally: fan.cancel(); 
      try: await fan; 
      except Exception: pass

  try: asyncio.run(run_async())
  except KeyboardInterrupt: pass
  finally:
    try: 
      if state.get("reader_stop"): state["reader_stop"].set()
    except Exception: pass
    try: 
      if state.get("serial"): state["serial"].close()
    except Exception: pass
    try: httpd.shutdown()
    except Exception: pass

if __name__ == "__main__":
  main()
