import sys
import argparse
from PyQt5 import QtWidgets, QtCore

from motor_manager import MotorManager
# ------------------------------------------------------------
# Which slaves are the steering/position motors?
# ------------------------------------------------------------
POSITION_MOTOR_SLAVES = [0, 2, 4, 6]


class ControlWindow(QtWidgets.QWidget):
    """4‑Wheel‑Steer control panel with PyQt5 sliders."""

    def __init__(self, iface: str = "eth0") -> None:
        super().__init__()
        self.setWindowTitle("4WS Motor Control Panel")
        self.iface = iface
        self.mm: MotorManager | None = None
        self.pos_per_deg: float = 1.0  # fallback – overwritten after EC init

        self._init_ethercat()   # Must come first – we query constants from lib
        self._build_ui()

    # -----------------------------------------------------------------
    # GUI construction
    # -----------------------------------------------------------------
    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # ── Slider for P0.332 – Smooth filter (ms) ────────────────────
        self.sld_0332 = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.sld_0332.setRange(0, 1000)
        self.sld_0332.setValue(0)
        self.lbl_0332 = QtWidgets.QLabel("P0.332 Smooth filter: 0 ms")
        self.sld_0332.valueChanged.connect(
            lambda v: self.lbl_0332.setText(f"P0.332 Smooth filter: {v} ms"))

        # ── Slider for P0.342 – FIR filter ────────────────────────────
        self.sld_0342 = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.sld_0342.setRange(0, 1000)
        self.sld_0342.setValue(0)
        self.lbl_0342 = QtWidgets.QLabel("P0.342 FIR filter: 0")
        self.sld_0342.valueChanged.connect(
            lambda v: self.lbl_0342.setText(f"P0.342 FIR filter: {v}"))

        # ── Slider for absolute wheel angle (°) ───────────────────────
        self.sld_pos = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.sld_pos.setRange(-180, 180)
        self.sld_pos.setValue(0)
        self.lbl_pos = QtWidgets.QLabel("Target position: 0 °")
        self.sld_pos.valueChanged.connect(
            lambda v: self.lbl_pos.setText(f"Target position: {v} °"))

        # ── Write‑all push‑button ─────────────────────────────────────
        self.btn_write = QtWidgets.QPushButton("WRITE ALL")
        self.btn_write.setStyleSheet("font-weight:bold; padding:8px 16px;")
        self.btn_write.clicked.connect(self._on_write_all)

        # Add widgets to the form
        for w in (self.lbl_0332, self.sld_0332,
                  self.lbl_0342, self.sld_0342,
                  self.lbl_pos,  self.sld_pos,
                  self.btn_write):
            layout.addWidget(w)

    # -----------------------------------------------------------------
    # EtherCAT initialisation
    # -----------------------------------------------------------------
    def _init_ethercat(self):
        try:
            self.mm = MotorManager(self.iface)
            self.mm.initialize()
            # Read constant provided by the library (27.7*105.26 by default)
            if hasattr(self.mm, "pos_per_deg"):
                self.pos_per_deg = float(self.mm.pos_per_deg)

            # All position motors → Position mode once
            for idx in POSITION_MOTOR_SLAVES:
                self.mm.configure_position_mode(idx)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "EtherCAT error",
                                           f"Failed to init EtherCAT: {exc}")
            sys.exit(1)

    # -----------------------------------------------------------------
    # Button callback – commit all three slider values
    # -----------------------------------------------------------------
    def _on_write_all(self):
        if not self.mm:
            return

        p0332_val = self.sld_0332.value()
        p0342_val = self.sld_0342.value()
        pos_deg   = self.sld_pos.value()

        # --- filter parameters go to slave‑0 via MotorManager helpers ---
        self.mm.set_position_smooth_filter(p0332_val, slave_index=0)
        self.mm.set_position_fir_filter(p0342_val,    slave_index=0)

        # --- broadcast absolute position to all steering wheels --------
        counts = int(pos_deg * self.pos_per_deg)
        for idx in POSITION_MOTOR_SLAVES:
            self.mm.set_position(idx, counts)

    # -----------------------------------------------------------------
    # Graceful shutdown – stop drives & close master
    # -----------------------------------------------------------------
    def closeEvent(self, event):
        if self.mm:
            self.mm.stop_all()
            self.mm.close()
        super().closeEvent(event)

# ---------------------------------------------------------------------
# main()
# ---------------------------------------------------------------------

def main():
    app = QtWidgets.QApplication(sys.argv)

    parser = argparse.ArgumentParser(description="4WS motor GUI (PyQt5)")
    parser.add_argument("--iface", default="eth0",
                        help="EtherCAT interface (default eth0)")
    args = parser.parse_args()

    win = ControlWindow(args.iface)
    win.resize(500, 300)
    win.show()

    sys.exit(app.exec_())

if __name__ == "__main__":  # pragma: no cover
    main()
