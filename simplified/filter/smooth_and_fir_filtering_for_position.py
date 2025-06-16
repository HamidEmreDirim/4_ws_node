#!/usr/bin/env python3
"""
PyQt5 GUI to tune SV‑DA200 series servo drives via EtherCAT.

─ Widgets ──────────────────────────────────────────────────────────────
• Slider 1  (0‑1000 ms)  →  P0.332  Smooth LPF of position command (slave‑0)
• Slider 2  (0‑1000 ms)  →  P0.342  FIR filter of position command   (slave‑0)
• Slider 3  (‑180 … +180 °) → Target position for slaves 0,2,4,6
• Button    "Write All"   → Sends all three values in one go

Edit the constants at the top (INTERFACE, COUNTS_PER_REV, CANopen indexes)
according to your hardware/manual before running.
Run with sudo if pysoem requires root NIC access:
    sudo python3 motor_gui.py
"""

import sys, struct, ctypes, math, signal
from PyQt5 import QtWidgets, QtCore
from motor_manager import MotorManager

# ─────────────────────────── User‑configurable constants ──────────────
INTERFACE = "enp3s0"           # ↯ Change to the NIC that talks EtherCAT
POSITION_SLAVES = [0, 2, 4, 6]  # Slaves that work in Position mode

# CANopen indexes for SV‑DA200 (verify in the manual!)
P0_332_IDX = 0x2021  # Smooth LPF of position command
P0_342_IDX = 0x2022  # FIR filter of position command

COUNTS_PER_REV = 100000         # Encoder counts per mechanical revolution
COUNTS_PER_DEG = (COUNTS_PER_REV / 360.0) * 105.26

# ───────────────────────────── Main Window ────────────────────────────
class MotorGui(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SV‑DA200 Filter & Position Tuner")
        self.resize(420, 240)

        # Initialise EtherCAT once; reuse throughout session
        try:
            self.mm = MotorManager(INTERFACE)
            self.mm.initialize()
            for idx in POSITION_SLAVES:
                self.mm.configure_position_mode(idx)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "EtherCAT init failed", str(e))
            raise

        # Build UI
        self._build_ui()

    # ------------------------------------------------------------------
    def _build_ui(self):
        self.sldr_lpf  = self._make_slider(0, 1000)         # P0.332
        self.sldr_fir  = self._make_slider(0, 1000)         # P0.342
        self.sldr_pos  = self._make_slider(-180, 180)       # target position

        self.lbl_lpf   = QtWidgets.QLabel()
        self.lbl_fir   = QtWidgets.QLabel()
        self.lbl_pos   = QtWidgets.QLabel()

        self._update_label(self.lbl_lpf, "P0.332 LPF", self.sldr_lpf.value(), "ms")
        self._update_label(self.lbl_fir, "P0.342 FIR", self.sldr_fir.value(), "ms")
        self._update_label(self.lbl_pos, "Target pos", self.sldr_pos.value(), "°")

        self.btn_write = QtWidgets.QPushButton("Write All")
        self.btn_write.clicked.connect(self.write_all)

        grid = QtWidgets.QGridLayout(self)
        grid.addWidget(self.lbl_lpf,  0, 0); grid.addWidget(self.sldr_lpf,  0, 1)
        grid.addWidget(self.lbl_fir,  1, 0); grid.addWidget(self.sldr_fir,  1, 1)
        grid.addWidget(self.lbl_pos,  2, 0); grid.addWidget(self.sldr_pos,  2, 1)
        grid.addWidget(self.btn_write,3, 0, 1, 2)

        # Live label updates
        self.sldr_lpf.valueChanged.connect(lambda v: self._update_label(self.lbl_lpf, "P0.332 LPF", v, "ms"))
        self.sldr_fir.valueChanged.connect(lambda v: self._update_label(self.lbl_fir, "P0.342 FIR", v, "ms"))
        self.sldr_pos.valueChanged.connect(lambda v: self._update_label(self.lbl_pos, "Target pos", v, "°"))

    # ------------------------------------------------------------------
    def _make_slider(self, lo:int, hi:int) -> QtWidgets.QSlider:
        s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        s.setRange(lo, hi)
        s.setTickInterval(max(1, (hi - lo) // 20))
        s.setTickPosition(QtWidgets.QSlider.TicksBelow)
        s.setSingleStep(1)
        s.setPageStep(10)
        return s

    @staticmethod
    def _update_label(lbl:QtWidgets.QLabel, name:str, val:int, unit:str):
        lbl.setText(f"{name}: {val} {unit}")

    # ------------------------------------------------------------------
    def _write_u16(self, slave_idx:int, index:int, value:int):
        """Write unsigned 16‑bit SDO."""
        try:
            payload = struct.pack("<H", value & 0xFFFF)
            self.mm.slaves[slave_idx].sdo_write(index, 0, payload)
            print(f"SDO 0x{index:04X}.0 <- {value} (slave {slave_idx})")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "SDO Error",
                                           f"Failed to write 0x{index:04X} on slave {slave_idx}: {e}")

    @staticmethod
    def _deg_to_counts(deg:float) -> int:
        return int(deg * COUNTS_PER_DEG)

    # ------------------------------------------------------------------
    def write_all(self):
        lpf_val = self.sldr_lpf.value()
        fir_val = self.sldr_fir.value()
        pos_deg = self.sldr_pos.value()
        tgt_cnt = self._deg_to_counts(pos_deg)

        # 1️⃣ Filters on slave‑0
        self._write_u16(0, P0_332_IDX, lpf_val)
        self._write_u16(0, P0_342_IDX, fir_val)

        # 2️⃣ Position on all designated slaves
        for idx in POSITION_SLAVES:
            self.mm.set_position(idx, tgt_cnt)

        QtWidgets.QMessageBox.information(self, "Done",
            f"LPF={lpf_val} ms, FIR={fir_val} ms, Target={pos_deg}° ({tgt_cnt} cnt)")

    # ------------------------------------------------------------------
    def closeEvent(self, event):
        """Graceful shutdown when window closed."""
        try:
            self.mm.stop_all()
            self.mm.close()
        except Exception:
            pass
        event.accept()

# ──────────────────────────────────────────────────────────────────────
def main():
    # Allow Ctrl‑C in terminal to kill GUI
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QtWidgets.QApplication(sys.argv)
    gui = MotorGui(); gui.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
