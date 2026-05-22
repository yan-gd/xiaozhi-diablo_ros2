#!/usr/bin/env python3
"""Pipe a local stdio MCP server to the xiaozhi.me MCP WebSocket endpoint.

This is a small dependency-free bridge for the robot. It implements enough of
RFC 6455 for xiaozhi's text-frame MCP endpoint: TLS WebSocket handshake, text
frames, close frames, and ping/pong.
"""

import argparse
import base64
import hashlib
import os
import random
import signal
import socket
import ssl
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from urllib.parse import urlparse


GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
OP_CONTINUATION = 0x0
OP_TEXT = 0x1
OP_CLOSE = 0x8
OP_PING = 0x9
OP_PONG = 0xA


def log(message: str) -> None:
    print("[xiaozhi_mcp_pipe] %s" % message, file=sys.stderr, flush=True)


@dataclass
class WebSocketUrl:
    secure: bool
    host: str
    port: int
    path: str


class SimpleWebSocket:
    def __init__(self, endpoint: str, timeout: float = 30.0) -> None:
        self.endpoint = endpoint
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self._send_lock = threading.Lock()

    def connect(self) -> None:
        parsed = self._parse_url(self.endpoint)
        raw_sock = socket.create_connection((parsed.host, parsed.port), timeout=self.timeout)
        if parsed.secure:
            context = ssl.create_default_context()
            self.sock = context.wrap_socket(raw_sock, server_hostname=parsed.host)
        else:
            self.sock = raw_sock
        self.sock.settimeout(self.timeout)
        self._handshake(parsed)
        # The endpoint can stay idle for a long time after registration. Keep
        # the socket blocking after the handshake so idle periods do not kill
        # the receive thread.
        self.sock.settimeout(None)

    def close(self) -> None:
        try:
            if self.sock is not None:
                try:
                    self._send_frame(OP_CLOSE, b"")
                except Exception:
                    pass
                self.sock.close()
        finally:
            self.sock = None

    def send_text(self, text: str) -> None:
        self._send_frame(OP_TEXT, text.encode("utf-8"))

    def recv_text(self) -> Optional[str]:
        fragments: List[bytes] = []
        while True:
            opcode, payload = self._recv_frame()
            if opcode == OP_TEXT:
                if fragments:
                    fragments.append(payload)
                    return b"".join(fragments).decode("utf-8")
                return payload.decode("utf-8")
            if opcode == OP_CONTINUATION:
                fragments.append(payload)
                continue
            if opcode == OP_PING:
                self._send_frame(OP_PONG, payload)
                continue
            if opcode == OP_PONG:
                continue
            if opcode == OP_CLOSE:
                self._send_frame(OP_CLOSE, b"")
                return None

    def _handshake(self, parsed: WebSocketUrl) -> None:
        assert self.sock is not None
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        host_header = parsed.host
        default_port = 443 if parsed.secure else 80
        if parsed.port != default_port:
            host_header = "%s:%d" % (parsed.host, parsed.port)
        request = (
            "GET {path} HTTP/1.1\r\n"
            "Host: {host}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        ).format(path=parsed.path, host=host_header, key=key)
        self.sock.sendall(request.encode("ascii"))
        response = self._read_http_response()
        status_line, headers = self._parse_http_response(response)
        if " 101 " not in status_line:
            raise RuntimeError("WebSocket handshake failed: %s" % status_line)
        accept = headers.get("sec-websocket-accept")
        expected = base64.b64encode(hashlib.sha1((key + GUID).encode("ascii")).digest()).decode("ascii")
        if accept != expected:
            raise RuntimeError("WebSocket handshake accept mismatch")

    def _read_http_response(self) -> bytes:
        assert self.sock is not None
        chunks = []
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = self.sock.recv(4096)
            if not chunk:
                break
            chunks.append(chunk)
            data = b"".join(chunks)
            if len(data) > 65536:
                raise RuntimeError("HTTP handshake response too large")
        return data

    def _parse_http_response(self, response: bytes) -> Tuple[str, Dict[str, str]]:
        text = response.decode("iso-8859-1", errors="replace")
        header_text = text.split("\r\n\r\n", 1)[0]
        lines = header_text.split("\r\n")
        status_line = lines[0] if lines else ""
        headers: Dict[str, str] = {}
        for line in lines[1:]:
            if ":" in line:
                key, value = line.split(":", 1)
                headers[key.strip().lower()] = value.strip()
        return status_line, headers

    def _send_frame(self, opcode: int, payload: bytes) -> None:
        assert self.sock is not None
        if len(payload) > 0xFFFFFFFF:
            raise ValueError("payload too large")
        first = 0x80 | opcode
        mask_bit = 0x80
        if len(payload) < 126:
            header = struct.pack("!BB", first, mask_bit | len(payload))
        elif len(payload) <= 0xFFFF:
            header = struct.pack("!BBH", first, mask_bit | 126, len(payload))
        else:
            header = struct.pack("!BBQ", first, mask_bit | 127, len(payload))
        mask = os.urandom(4)
        masked = bytes(byte ^ mask[i % 4] for i, byte in enumerate(payload))
        with self._send_lock:
            self.sock.sendall(header + mask + masked)

    def _recv_frame(self) -> Tuple[int, bytes]:
        assert self.sock is not None
        header = self._read_exact(2)
        first, second = struct.unpack("!BB", header)
        fin = bool(first & 0x80)
        opcode = first & 0x0F
        masked = bool(second & 0x80)
        length = second & 0x7F
        if length == 126:
            length = struct.unpack("!H", self._read_exact(2))[0]
        elif length == 127:
            length = struct.unpack("!Q", self._read_exact(8))[0]
        mask = self._read_exact(4) if masked else b""
        payload = self._read_exact(length) if length else b""
        if masked:
            payload = bytes(byte ^ mask[i % 4] for i, byte in enumerate(payload))
        if not fin and opcode not in (OP_TEXT, OP_CONTINUATION):
            raise RuntimeError("fragmented control frame is invalid")
        return opcode, payload

    def _read_exact(self, length: int) -> bytes:
        assert self.sock is not None
        data = b""
        while len(data) < length:
            chunk = self.sock.recv(length - len(data))
            if not chunk:
                raise ConnectionError("socket closed")
            data += chunk
        return data

    def _parse_url(self, endpoint: str) -> WebSocketUrl:
        parsed = urlparse(endpoint)
        if parsed.scheme not in ("ws", "wss"):
            raise ValueError("MCP_ENDPOINT must start with ws:// or wss://")
        if not parsed.hostname:
            raise ValueError("MCP_ENDPOINT is missing host")
        secure = parsed.scheme == "wss"
        port = parsed.port or (443 if secure else 80)
        path = parsed.path or "/"
        if parsed.query:
            path += "?" + parsed.query
        return WebSocketUrl(secure=secure, host=parsed.hostname, port=port, path=path)


class PipeRunner:
    def __init__(self, endpoint: str, command: List[str]) -> None:
        self.endpoint = endpoint
        self.command = command
        self.stop_event = threading.Event()
        self._cycle_failed = threading.Event()
        self._cycle_error = "unknown bridge error"
        self.process: Optional[subprocess.Popen[str]] = None
        self.ws: Optional[SimpleWebSocket] = None

    def run_forever(self) -> None:
        backoff = 1.0
        while not self.stop_event.is_set():
            try:
                self._run_once()
                backoff = 1.0
            except Exception as exc:
                if self.stop_event.is_set():
                    break
                log("connection ended: %s" % exc)
                sleep_for = min(backoff, 60.0) + random.uniform(0.0, 0.3)
                log("reconnecting in %.1fs" % sleep_for)
                self.stop_event.wait(sleep_for)
                backoff = min(backoff * 2.0, 60.0)
            finally:
                self._cleanup()

    def stop(self) -> None:
        self.stop_event.set()
        self._cleanup()

    def _run_once(self) -> None:
        log("connecting to MCP endpoint")
        self.ws = SimpleWebSocket(self.endpoint)
        self.ws.connect()
        log("connected")
        self.process = subprocess.Popen(
            self.command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            bufsize=1,
            start_new_session=True,
        )
        log("started local MCP server: %s" % " ".join(self.command))
        self._cycle_failed.clear()
        self._cycle_error = "unknown bridge error"

        threads = [
            threading.Thread(target=self._endpoint_to_process, daemon=True),
            threading.Thread(target=self._process_to_endpoint, daemon=True),
            threading.Thread(target=self._process_stderr_to_terminal, daemon=True),
        ]
        for thread in threads:
            thread.start()
        while not self.stop_event.is_set():
            if self.process.poll() is not None:
                raise RuntimeError("local MCP process exited with code %s" % self.process.returncode)
            if self._cycle_failed.is_set():
                raise RuntimeError(self._cycle_error)
            time.sleep(0.2)

    def _endpoint_to_process(self) -> None:
        try:
            assert self.ws is not None
            assert self.process is not None
            assert self.process.stdin is not None
            while not self.stop_event.is_set():
                message = self.ws.recv_text()
                if message is None:
                    raise RuntimeError("endpoint closed")
                self.process.stdin.write(message.rstrip("\n") + "\n")
                self.process.stdin.flush()
        except Exception as exc:
            self._mark_cycle_failed("endpoint->process failed: %s" % exc)

    def _process_to_endpoint(self) -> None:
        try:
            assert self.ws is not None
            assert self.process is not None
            assert self.process.stdout is not None
            while not self.stop_event.is_set():
                line = self.process.stdout.readline()
                if not line:
                    raise RuntimeError("local MCP stdout closed")
                self.ws.send_text(line.rstrip("\n"))
        except Exception as exc:
            self._mark_cycle_failed("process->endpoint failed: %s" % exc)

    def _process_stderr_to_terminal(self) -> None:
        assert self.process is not None
        assert self.process.stderr is not None
        while not self.stop_event.is_set():
            line = self.process.stderr.readline()
            if not line:
                return
            sys.stderr.write(line)
            sys.stderr.flush()

    def _mark_cycle_failed(self, message: str) -> None:
        if self.stop_event.is_set():
            return
        self._cycle_error = message
        self._cycle_failed.set()

    def _cleanup(self) -> None:
        if self.ws is not None:
            self.ws.close()
            self.ws = None
        if self.process is not None:
            if self.process.poll() is None:
                self._terminate_process_group()
            self.process = None

    def _terminate_process_group(self) -> None:
        assert self.process is not None
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            self.process.wait(timeout=5)
            return
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
        self.process.wait(timeout=5)


def main() -> None:
    parser = argparse.ArgumentParser(description="Dependency-free xiaozhi MCP endpoint pipe")
    parser.add_argument("command", nargs=argparse.REMAINDER, help="Local MCP stdio command to run")
    parser.add_argument(
        "--endpoint",
        default=os.environ.get("MCP_ENDPOINT"),
        help="xiaozhi.me MCP endpoint. Defaults to MCP_ENDPOINT.",
    )
    args = parser.parse_args()

    if not args.endpoint:
        print("Set MCP_ENDPOINT or pass --endpoint.", file=sys.stderr)
        sys.exit(2)
    command = args.command or [sys.executable, "-m", "xiaozhi_robot_control.robot_mcp_server"]
    if command and command[0] == "--":
        command = command[1:]

    runner = PipeRunner(args.endpoint, command)

    def handle_signal(signum, frame):  # type: ignore[no-untyped-def]
        log("received signal %s, shutting down" % signum)
        runner.stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    runner.run_forever()


if __name__ == "__main__":
    main()
