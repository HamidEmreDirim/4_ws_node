import pysoem
import ctypes
import time
import struct

# Dönüş yönleri ters olan motor indekslerini buraya ekleyebilirsiniz
reverse_motors =  [0, 1, 4, 3, 5, 7]

class MotorManager:
    def __init__(self, interface_name):
        self.interface_name = interface_name
        self.master = pysoem.Master()
        self.slaves = []
        self.open()

    def open(self):
        self.master.open(self.interface_name)
        print(f"EtherCAT master opened on {self.interface_name}.")

    def initialize(self):
        if self.master.config_init() > 0:
            self.slaves = self.master.slaves

            print(f"Found {len(self.slaves)} slave(s).")
            if len(self.slaves) < 1:
                raise Exception("No slaves found.")

            self.master.config_map()
            self.master.state = pysoem.SAFEOP_STATE
            self.master.write_state()
            # Her slave SAFEOP_STATE'e geçene kadar bekle
            for i in range(100):
                self.master.read_state()
                if all(sl.state == pysoem.SAFEOP_STATE for sl in self.slaves):
                    break
                time.sleep(0.01)

            if not all(sl.state == pysoem.SAFEOP_STATE for sl in self.slaves):
                raise Exception("Failed to bring all slaves into SAFEOP_STATE.")
        else:
            raise Exception("Failed to initialize EtherCAT network.")

    def close(self):
        self.master.close()
        print("EtherCAT master closed.")

    #----------------------
    # Position Mode Methods
    #----------------------
    def configure_position_mode(self, slave_index):
        """Belirtilen slave'i pozisyon moduna alır ve operasyonel hale getirir."""
        try:
            slave = self.slaves[slave_index]
            # Mode of Operation: Position Mode = 7
            slave.sdo_write(0x6060, 0, bytes(ctypes.c_int8(7)))  
            time.sleep(0.1)

            # Shutdown state
            slave.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(0x0006))) 
            time.sleep(0.1)
            # Switch on
            slave.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(0x0007))) 
            time.sleep(0.1)
            # Enable operation
            slave.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(0x000F))) 
            time.sleep(0.1)

            print(f"Slave {slave_index} configured in Position Mode.")
        except Exception as e:
            print(f"Error configuring slave {slave_index} in Position Mode: {e}")

    def set_position(self, slave_index, target_position):
        """Hedef pozisyonu (Target Position) ayarlar."""
        try:
            slave = self.slaves[slave_index]
            # Hedef pozisyonu yaz
            if (slave_index in reverse_motors):
                target_position = target_position * -1
            slave.sdo_write(0x607A, 0, bytes(ctypes.c_int32(target_position)))

            # "New set-point" bitini tetiklemek için...
            slave.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(0x001F))) 
            slave.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(0x000F))) 
            print(f"Set position of slave {slave_index} to {target_position}.")
        except Exception as e:
            print(f"Error setting position for slave {slave_index}: {e}")

    #----------------------
    # Velocity Mode Methods
    #----------------------
    def configure_velocity_mode(self, slave_index):
        try:
            slave = self.slaves[slave_index]
            # Velocity Mode = 3
            slave.sdo_write(0x6060, 0, bytes(ctypes.c_int8(3)))
            time.sleep(0.05)

            # Max velocity örneği (isteğe göre değiştirin)
            slave.sdo_write(0x6046, 2, bytes(ctypes.c_int16(3000)))
            time.sleep(0.05)

            # Switch on, enable
            slave.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(0x0007)))
            time.sleep(0.05)
            slave.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(0x000F)))
            time.sleep(0.05)

            print(f"Slave {slave_index} configured in Velocity Mode.")
        except Exception as e:
            print(f"Error configuring slave {slave_index} in Velocity Mode: {e}")

    def set_velocity(self, slave_index, target_velocity):
        """Velocity komutu gönderir. Sisteme göre sınırları ayarlayabilirsiniz."""
        try:
            # Güvenlik amaçlı basit limit
            if target_velocity > 500:
                target_velocity = 500
            if target_velocity < -500:
                target_velocity = -500

            slave = self.slaves[slave_index]
            if slave_index in reverse_motors:
                target_velocity = target_velocity * -1

            slave.sdo_write(0x60FF, 0, bytes(ctypes.c_int32(target_velocity)))
            print(f"Set velocity of slave {slave_index} to {target_velocity}.")
        except Exception as e:
            print(f"Error setting velocity for slave {slave_index}: {e}")

    def stop_all(self):
        """Tüm motorları durdur."""
        for i, slave in enumerate(self.slaves):
            try:
                slave.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(0x0002)))  # Disable operation
                slave.sdo_write(0x6040, 0, bytes(ctypes.c_uint16(0x0000)))  # Shutdown
                print(f"Stopped slave {i}.")
            except Exception as e:
                print(f"Error stopping slave {i}: {e}")

    # Buradan sonrası, SDO okuma vs. (aynı kalabilir). İhtiyaç halinde kullanılır...
    # -----------------------------------------------------------------
    def _read_sdo_16bit_signed(self, slave_index, index, subindex=0):
        try:
            data = self.slaves[slave_index].sdo_read(index, subindex)
            if data is None or len(data) < 2:
                raise ValueError(f"Expected at least 2 bytes, got {len(data) if data else 0} bytes.")
            if len(data) == 2:
                value = struct.unpack("<h", data)[0]  # signed 16-bit
                return value
            elif len(data) == 4:
                value = struct.unpack("<i", data)[0]  # signed 32-bit
                return value
            else:
                raise ValueError(f"Unexpected data length: {len(data)} bytes. Data: {data}")
        except Exception as e:
            print(f"Error reading 16-bit signed parameter {index}:{subindex} for slave {slave_index}: {e}")
            return None

    def _read_sdo_32bit_signed(self, slave_index, index, subindex=0):
        try:
            data = self.slaves[slave_index].sdo_read(index, subindex)
            if len(data) < 4:
                raise ValueError(f"Expected 4 bytes, got {len(data)} bytes.")
            return struct.unpack("<i", data)[0]  # 32-bit signed
        except Exception as e:
            print(f"Error reading 32-bit signed parameter {index}:{subindex}: {e}")
            return None

    def get_current_torque(self, slave_index):
        try:
            raw_value = self._read_sdo_16bit_signed(slave_index, 0x3006, 0x00)
            if raw_value is not None:
                return raw_value / 10.0
            return None
        except Exception as e:
            print(f"Error reading R0.06_Current_Torque for slave {slave_index}: {e}")
            return None

    # vs...
