#!/usr/bin/env python3
"""
Gerçek motor ECU’su yerine geçer. Gelen her JSON satırını ekrana yazar.
"""

import json, socket

HOST, PORT = '0.0.0.0', 9000
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    print(f'🔌  Listening on {HOST}:{PORT} …')
    conn, addr = s.accept()
    with conn:
        print('✅  Connection from', addr)
        buf = b''
        while True:
            data = conn.recv(1024)
            if not data:
                print('💔  Client closed connection'); break
            buf += data
            while b'\n' in buf:                 # satırlara ayır
                line, buf = buf.split(b'\n', 1)
                try:
                    msg = json.loads(line)
                    print('📨', msg)
                except Exception as e:
                    print('⚠️  Bad JSON:', line, e)
