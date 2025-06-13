#!/usr/bin/env python3

import socket
import json
import threading
import time
from motor_manager import MotorManager  # Sizin motor sürücü sınıfınız

class MotorServer:
    def __init__(self, host='0.0.0.0', port=9000, interface='enp3s0'):
        self.host = host
        self.port = port
        self.interface = interface

        # EtherCAT Motor Manager başlat
        self.motor_manager = MotorManager(self.interface)
        self.motor_manager.initialize()

        # -----------------------------------------------------------
        # 8 motoru tek tek konfigüre edelim:
        #   0,2,4,6 -> Position mode
        #   1,3,5,7 -> Velocity mode
        # -----------------------------------------------------------
        for i in range(8):
            try:
                if i in [0, 2, 4, 6]:
                    self.motor_manager.configure_position_mode(i)
                else:
                    self.motor_manager.configure_velocity_mode(i)
            except Exception as e:
                print(f"[MotorServer Debug] Error configuring slave {i}: {e}")

    def start(self):
        """TCP sunucusunu başlatır ve gelen bağlantıları dinler."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen(5)
            print(f"[MotorServer] Listening on {self.host}:{self.port} ...")

            while True:
                conn, addr = s.accept()
                print(f"[MotorServer] Client connected: {addr}")
                threading.Thread(target=self.handle_client, args=(conn,)).start()

    def handle_client(self, conn):
        """
        Her istemci için gelen JSON mesajlarını *satır bazlı* okur.
        Tek TCP recv ile birden çok JSON gelebileceği için, \n ile böleceğiz.
        """
        with conn:
            buffer = ""
            while True:
                data = conn.recv(4096)
                if not data:
                    break  # Bağlantı koptuysa sonlandır
                buffer += data.decode('utf-8')

                # Satırlara ayır
                lines = buffer.split('\n')
                # Son satır dışında kalanların her biri tam bir JSON kabul
                for line in lines[:-1]:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                    except Exception as e:
                        print(f"[MotorServer] JSON parse error: {e}")
                        continue

                    # Komutları ayrıştır
                    command = msg.get('command', None)
                    slave_index = msg.get('index', None)
                    value = msg.get('value', None)

                    # Debug: Gelen komutu ekrana bas
                    print(f"[MotorServer Debug] Received: {msg}")

                    if command == 'set_position':
                        self.motor_manager.set_position(slave_index, value)
                    elif command == 'set_velocity':
                        self.motor_manager.set_velocity(slave_index, value)
                    elif command == 'stop_all':
                        self.motor_manager.stop_all()
                    else:
                        print(f"[MotorServer] Unknown command: {command}")

                # buffer'ı elde kalan (henüz \n gelmemiş) satırla güncelle
                buffer = lines[-1]

def main():
    # Bu script'i sudo ile çalıştırın (ör: sudo python3 motor_server.py)
    server = MotorServer(host='0.0.0.0', port=9000, interface='enp3s0')
    server.start()

if __name__ == '__main__':
    main()
