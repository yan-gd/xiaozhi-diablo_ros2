#!/usr/bin/env python3
"""Small local stdio smoke test for the robot MCP server."""

import json
import os
import subprocess
import sys
import time


def request(proc, payload):
    proc.stdin.write(json.dumps(payload) + "\n")
    proc.stdin.flush()
    return json.loads(proc.stdout.readline())


def main():
    tool_name = sys.argv[1] if len(sys.argv) > 1 else "robot_stop"
    tool_arguments = {}
    if len(sys.argv) > 2:
        tool_arguments = json.loads(sys.argv[2])

    script_dir = os.path.dirname(os.path.abspath(__file__))
    server = os.path.join(script_dir, "run_robot_mcp_stdio.sh")
    proc = subprocess.Popen(
        [server],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=sys.stderr,
        text=True,
    )
    try:
        print(request(proc, {"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {}}))
        print(request(proc, {"jsonrpc": "2.0", "id": 2, "method": "tools/list"}))
        print(
            request(
                proc,
                {
                    "jsonrpc": "2.0",
                    "id": 3,
                    "method": "tools/call",
                    "params": {"name": tool_name, "arguments": tool_arguments},
                },
            )
        )
        time.sleep(1.0)
    finally:
        proc.terminate()
        proc.wait(timeout=5)


if __name__ == "__main__":
    main()
