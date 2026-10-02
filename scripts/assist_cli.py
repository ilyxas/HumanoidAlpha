#!/usr/bin/env python3
"""Thin ASSISTED-stand CLI. Does not change ./humanoid (RAW act/zero only)."""
import json
import os
import socket
import sys


OPS = ('on', 'off', 'reset', 'status')


def main():
    args = sys.argv[1:]
    if len(args) != 1 or args[0] not in OPS:
        print('ERROR use: humanoid-assist on|off|reset|status', file=sys.stderr)
        return 2
    path = os.environ.get('HUMANOID_ACTUATOR_SOCKET', f'/tmp/humanoid-actuator-{os.getuid()}.sock')
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(5)
            client.connect(path)
            client.sendall((json.dumps({'assist': args[0]}, allow_nan=False) + '\n').encode())
            reply = client.makefile('rb').readline(4096).decode().strip()
        if reply.startswith('OK'):
            print(reply)
            return 0
        print(reply or 'ERROR invalid or busy request', file=sys.stderr)
    except (OSError, UnicodeError):
        print('ERROR runtime unavailable; command outcome unknown', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
