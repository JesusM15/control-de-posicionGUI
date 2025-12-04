import sys
import serial
from collections import deque

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

PORT = 'COM5'
BAUD = 9600
WINDOW_SECONDS = 5.0
MAX_POINTS = 3000
DOWNSAMPLE_TARGET = 1200

class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control de posición")
        self.resize(900, 580)

        self.current_error = 0.0

        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.0)
        except serial.SerialException as e:
            self.ser = None
            print(f"[WARN] No se pudo abrir {PORT}: {e}")

        top = QtWidgets.QVBoxLayout()

        refRow = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel("Referencia (°):")
        self.refEdit = QtWidgets.QLineEdit("0.0")
        self.refEdit.setFixedWidth(80)
        self.sendBtn = QtWidgets.QPushButton("Enviar referencia")

        self.referencia = QtWidgets.QLabel("Referencia: 0")
        self.status = QtWidgets.QLabel("Listo")
        self.status.setStyleSheet("color: #56DB14;")
        self.referencia.setStyleSheet("color: #fff;")

        refRow.addWidget(lbl)
        refRow.addWidget(self.refEdit)
        refRow.addWidget(self.sendBtn)
        refRow.addSpacing(15)
        refRow.addWidget(self.status, 1)
        refRow.addWidget(self.referencia, 2)

        top.addLayout(refRow)

        pidRow = QtWidgets.QHBoxLayout()
        pidRow.addWidget(QtWidgets.QLabel("kp:"))
        self.kpEdit = QtWidgets.QLineEdit("3.05")
        self.kpEdit.setFixedWidth(70)
        pidRow.addWidget(self.kpEdit)

        pidRow.addWidget(QtWidgets.QLabel("ki:"))
        self.kiEdit = QtWidgets.QLineEdit("0.1")
        self.kiEdit.setFixedWidth(70)
        pidRow.addWidget(self.kiEdit)

        pidRow.addWidget(QtWidgets.QLabel("kd:"))
        self.kdEdit = QtWidgets.QLineEdit("4.6")
        self.kdEdit.setFixedWidth(70)
        pidRow.addWidget(self.kdEdit)

        self.sendPIDBtn = QtWidgets.QPushButton("Enviar PID")
        pidRow.addWidget(self.sendPIDBtn)
        pidRow.addStretch(1)
        top.addLayout(pidRow)

        modeRow = QtWidgets.QHBoxLayout()

        self.rb_closed = QtWidgets.QRadioButton("Lazo cerrado")
        self.rb_open   = QtWidgets.QRadioButton("Lazo abierto")

        self.rb_closed.setChecked(True)

        modeRow.addWidget(self.rb_closed)
        modeRow.addWidget(self.rb_open)
        modeRow.addStretch(1)

        top.addLayout(modeRow)

        self.rb_closed.toggled.connect(self.onModeChanged)
        self.rb_open.toggled.connect(self.onModeChanged)

        self.errorBox = QtWidgets.QLabel("Error actual: 0.00°")
        self.errorBox.setStyleSheet("""
            background-color: #ffffff;
            color: #000000;
            border-radius: 6px;
            border: 1px solid #555;
            font-size: 12px;
            font-weight: bold;
        """)
        self.errorBox.setFixedWidth(130)
        self.errorBox.setFixedHeight(45)
        top.addWidget(self.errorBox)

        self.plot = pg.PlotWidget(background="#0e1116")
        self.plot.setTitle("Ángulo vs Tiempo", color="#e0e0e0")
        self.plot.setLabel('left', 'Ángulo / Referencia (°)')
        self.plot.setLabel('bottom', 'Tiempo (s)')
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setYRange(-360.0, 360.0)

        self.curve_angle = self.plot.plot([], [], pen=pg.mkPen('#4da3ff', width=2))
        self.curve_ref   = self.plot.plot([], [], pen=pg.mkPen('#ff5555', width=2, style=QtCore.Qt.DashLine))

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(self.plot)

        self.t0 = None
        self.x = deque(maxlen=MAX_POINTS)
        self.y = deque(maxlen=MAX_POINTS)
        self.y_ref = deque(maxlen=MAX_POINTS)
        self._rx_buf = bytearray()
        self.current_ref = 0.0

        self.sendBtn.clicked.connect(self.onSendRef)
        self.refEdit.returnPressed.connect(self.onSendRef)

        self.sendPIDBtn.clicked.connect(self.onSendPID)
        self.kpEdit.returnPressed.connect(self.onSendPID)
        self.kiEdit.returnPressed.connect(self.onSendPID)
        self.kdEdit.returnPressed.connect(self.onSendPID)

        self.readTimer = QtCore.QTimer(self)
        self.readTimer.timeout.connect(self.readSerialBatch)
        self.readTimer.start(2)

        self.plotTimer = QtCore.QTimer(self)
        self.plotTimer.timeout.connect(self.updatePlot)
        self.plotTimer.start(8)

        self.sendMode(0)

    def onModeChanged(self, checked: bool):
        if not checked:
            return

        if self.rb_closed.isChecked():
            mode = 0  
        else:
            mode = 1  

        self.sendMode(mode)

    def sendMode(self, mode: int):
        if not self.ser:
            # self.status.setText("Sin conexión serial")
            # self.status.setStyleSheet("color: #ff6b6b;")
            return

        cmd = f"m={mode}\n"
        try:
            self.ser.write(cmd.encode('utf-8'))
            text = "Lazo cerrado" if mode == 0 else "Lazo abierto"
            pass
            # self.status.setText(f"TX modo: {text} ({cmd.strip()})")
            # self.status.setStyleSheet("color: #7ad12b;")
        except Exception as e:
            pass
            # self.status.setText(f"Error TX modo: {e}")
            # self.status.setStyleSheet("color: #ff6b6b;")

    def readSerialBatch(self):
        if not self.ser:
            return
        try:
            n = self.ser.in_waiting
            if n:
                chunk = self.ser.read(n)
                self._rx_buf.extend(chunk)
                while True:
                    nl = self._rx_buf.find(b'\n')
                    if nl < 0:
                        break
                    line = self._rx_buf[:nl]
                    del self._rx_buf[:nl + 1]
                    self._processLine(line)
        except Exception as e:
            pass
            # self.status.setText(f"Error lectura: {e}")
            # self.status.setStyleSheet("color: #ff6b6b;")

    def _processLine(self, line: bytes):
        try:
            s = line.decode('utf-8', errors='ignore').strip()
            if not s:
                return

            parts = s.split(',')
            if len(parts) < 2:
                return

            ang = float(parts[0])

            if len(parts) >= 3:
                t = float(parts[2])
            else:
                t = float(parts[1])

            self.current_error = self.current_ref - ang
            t_s = t / 1000.0
            self.status.setText(f'Angulo Real {ang}')
            self._pushSample(t_s, ang)
        except ValueError:
            pass

    def _pushSample(self, t_abs, ang):
        if self.t0 is None:
            self.t0 = t_abs
        t_rel = t_abs - self.t0

        self.x.append(t_rel)
        self.y.append(ang)
        self.y_ref.append(self.current_ref)

        xmin = t_rel - WINDOW_SECONDS
        while self.x and self.x[0] < xmin:
            self.x.popleft()
            self.y.popleft()
            self.y_ref.popleft()

    def updatePlot(self):
        if not self.x:
            return

        step = max(1, len(self.x) // DOWNSAMPLE_TARGET)

        xs = list(self.x)[::step]
        ys = list(self.y)[::step]
        yrefs = list(self.y_ref)[::step]

        self.curve_angle.setData(xs, ys)
        self.curve_ref.setData(xs, yrefs)
        self.plot.setXRange(self.x[0], self.x[-1], padding=0)

        self.errorBox.setText(f"Error actual: {self.current_error:.2f}°")

    def onSendRef(self):
        txt = self.refEdit.text().strip()
        try:
            ref = float(txt)
            self.referencia.setText(f'Referencia: {txt}')
        except ValueError:
            # self.status.setText("Referencia inválida")
            # self.status.setStyleSheet("color: #ff6b6b;")
            return

        self.current_ref = ref

        if not self.ser:
#            self.status.setText("Sin conexión serial")
            # self.status.setStyleSheet("color: #ff6b6b;")
            return

        try:
            self.ser.write((txt + "\n").encode('utf-8'))
            # self.status.setText(f"TX ref: {txt}")
#            self.status.setStyleSheet("color: #7ad12b;")
        except Exception as e:
            pass
            # self.status.setText(f"Error TX ref: {e}")
            # self.status.setStyleSheet("color: #ff6b6b;")

    def onSendPID(self):
        kp_txt = self.kpEdit.text().strip()
        ki_txt = self.kiEdit.text().strip()
        kd_txt = self.kdEdit.text().strip()

        try:
            float(kp_txt); float(ki_txt); float(kd_txt)
        except ValueError:
            # self.status.setText("kp/ki/kd inválidos")
            # self.status.setStyleSheet("color: #ff6b6b;")
            return

        if not self.ser:
            # self.status.setText("Sin conexión serial")
            # self.status.setStyleSheet("color: #ff6b6b;")
            return

        cmd = f"kd={kd_txt},ki={ki_txt},kp={kp_txt}\n"
        try:
            self.ser.write(cmd.encode('utf-8'))
            # self.status.setText(f"TX PID: {cmd.strip()}")
            # self.status.setStyleSheet("color: #7ad12b;")
        except Exception as e:
            pass
            # self.status.setText(f"Error TX PID: {e}")
            # self.status.setStyleSheet("color: #ff6b6b;")

    def closeEvent(self, event):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except:
            pass
        event.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=False, useOpenGL=False)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
