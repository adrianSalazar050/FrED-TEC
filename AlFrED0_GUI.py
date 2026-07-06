# LIBRARIES RQUIERED:

import sys
import platform
import serial
import serial.tools.list_ports
import numpy as np
import csv
import cv2
import time

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QFileDialog, QLineEdit, QFormLayout, QGroupBox,
    QCheckBox, QDoubleSpinBox, QGridLayout, QScrollArea, QSizePolicy,
    QFrame, QSplitter
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage, QFont

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


# CLASS FOR PID CASCADE CONTROL OF DIAMETER

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=1.75, output_limits=(10, 30)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_output = None

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        if self.output_limits[0] is not None:
            output = max(output, self.output_limits[0])
        if self.output_limits[1] is not None:
            output = min(output, self.output_limits[1])

        if self.last_output is not None:
            if (output <= self.output_limits[0] or output >= self.output_limits[1]) and \
               self.output_limits[0] is not None and self.output_limits[1] is not None:
                self.integral -= error * dt * 0.5

        self.last_output = output
        return output


# SERIAL CONNECTION

def encontrar_puerto_arduino():
    puertos = serial.tools.list_ports.comports()
    for p in puertos:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if ("arduino" in desc) or ("usb serial" in desc) or ("ch340" in desc) or ("1a86" in hwid) or ("2341" in hwid):
            return p.device

    if platform.system() == "Windows": return "COM7"
    elif platform.system() == "Linux": return "/dev/ttyACM0"
    elif platform.system() == "Darwin": return "/dev/tty.usbmodem1101"
    else: return None

try:
    puerto = encontrar_puerto_arduino() or "/dev/ttyACM0"
    arduino = serial.Serial(puerto, 115200, timeout=1)
except Exception as e:
    print(f"Error opening serial port: {e}")
    sys.exit(1)


# CANVAS FOR PLOTS

class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=2, dpi=90):
        fig = Figure(figsize=(width, height), dpi=dpi)
        fig.subplots_adjust(left=0.12, right=0.97, top=0.93, bottom=0.15)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)
        self.setParent(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def plot(self, data, ylabel=""):
        self.axes.cla()
        if len(data) > 0:
            self.axes.plot(data, linestyle='-', linewidth=1.2, color='#2980b9')
        if ylabel:
            self.axes.set_ylabel(ylabel, fontsize=8)
        self.axes.tick_params(labelsize=7)
        self.axes.grid(True, alpha=0.4)
        self.draw()


# FUNCTION FOR SEPARATOR IN UI

def make_separator():
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setFrameShadow(QFrame.Sunken)
    line.setStyleSheet("color: #ccc;")
    return line


# COMPLETE GUI CLASS

class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AI-FrED0 Control Interface with Cascade Diameter Control")
        self.resize(1400, 860)

        # STATES OF THE UI
        self.estado = ['0', '0', '0', '0']
        self.velocidad_extrusor = 100
        self.temperatura_objetivo = 190
        self.velocidad_dc_objetivo = 20
        self.velocidad_fan = 0

        self.temp_data = []
        self.motor_rpm_data = []
        self.grosor_data = []
        self.motor_dc_state_data = []
        self.fan_state_data = []
        self.extruder_state_data = []
        self.heater_state_data = []

        self.FACTOR_CONVERSION = 0.4 / 255.0
        self.VALOR_UMBRAL = 5
        self.actuator_names = ["Motor DC", "Fan", "Extrusor", "Heater"]

        self.pid_diameter = PID(Kp=2.0, Ki=0.5, Kd=0.1, setpoint=1.75, output_limits=(10, 30))
        self.diameter_control_enabled = False
        self.last_pid_time = time.monotonic()
        self.last_rpm_setpoint = 20.0
        self.saved_manual_values = {}

        # BUILD UI
        self._build_ui()

        # CAMERA
        if platform.system() == "Windows":
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.label_camara.setText("Camera not available")

        self.timer_camara = QTimer()
        self.timer_camara.timeout.connect(self.actualizar_imagen_camara)
        self.timer_camara.start(50)

        self.timer = QTimer()
        self.timer.timeout.connect(self.actualizar)
        self.timer.start(250)

        for i in range(4):
            self.actualizar_indicador_estado(i)

    def _build_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(8)

        # LEFT PANEL: plots + info
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(4)

        self.canvas_temp  = PlotCanvas(self, width=6, height=2)
        self.canvas_motor = PlotCanvas(self, width=6, height=2)
        self.canvas_grosor = PlotCanvas(self, width=6, height=2)

        for canvas in (self.canvas_temp, self.canvas_motor, self.canvas_grosor):
            left_layout.addWidget(canvas, 1)

        # BOTTOM INFORMATION
        bottom_row = QWidget()
        bottom_row_layout = QHBoxLayout(bottom_row)
        bottom_row_layout.setContentsMargins(4, 4, 4, 4)

        # ACTUATOR INDICATORS
        status_box = QGroupBox("Actuator Status")
        status_box.setStyleSheet("QGroupBox { font-weight: bold; font-size: 13px; }")
        status_box_layout = QVBoxLayout(status_box)
        status_box_layout.setSpacing(6)

        self.indicator_colors = []
        self.indicator_labels = []
        for name in self.actuator_names:
            row = QHBoxLayout()
            dot = QLabel()
            dot.setFixedSize(18, 18)
            self.indicator_colors.append(dot)
            lbl = QLabel(f"{name}: OFF")
            lbl.setStyleSheet("font-size: 12px;")
            self.indicator_labels.append(lbl)
            row.addWidget(dot)
            row.addWidget(lbl)
            row.addStretch()
            status_box_layout.addLayout(row)

        bottom_row_layout.addWidget(status_box)

        # DIAMETER DISPLAY
        grosor_box = QGroupBox("Filament Diameter")
        grosor_box.setStyleSheet("QGroupBox { font-weight: bold; font-size: 13px; }")
        grosor_box_layout = QVBoxLayout(grosor_box)
        self.label_grosor_display = QLabel("-- mm")
        self.label_grosor_display.setStyleSheet(
            "font-weight: bold; font-size: 28px; color: #2980b9;"
        )
        self.label_grosor_display.setAlignment(Qt.AlignCenter)
        grosor_box_layout.addWidget(self.label_grosor_display)
        bottom_row_layout.addWidget(grosor_box)

        left_layout.addWidget(bottom_row)
        main_layout.addWidget(left_widget, 5)

        # RIGHT PANNEL: controls + camera
        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        right_scroll.setMinimumWidth(360)
        right_scroll.setMaximumWidth(440)
        right_scroll.setStyleSheet("QScrollArea { border: none; }")

        right_inner = QWidget()
        right_layout = QVBoxLayout(right_inner)
        right_layout.setContentsMargins(8, 8, 8, 8)
        right_layout.setSpacing(10)

        # ACTUATOR BUTTONS
        act_group = QGroupBox("Actuators")
        act_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        act_layout = QGridLayout(act_group)
        act_layout.setSpacing(6)

        self.btn_spool   = QPushButton(f"{self.actuator_names[0]} (OFF)")
        self.btn_fan     = QPushButton(f"{self.actuator_names[1]} (OFF)")
        self.btn_extrude = QPushButton(f"{self.actuator_names[2]} (OFF)")
        self.btn_heater  = QPushButton(f"{self.actuator_names[3]} (OFF)")
        self.actuator_buttons = [self.btn_spool, self.btn_fan, self.btn_extrude, self.btn_heater]

        btn_style = """
            QPushButton {
                padding: 6px; border-radius: 4px;
                background-color: #e74c3c; color: white; font-weight: bold;
            }
            QPushButton:checked { background-color: #27ae60; }
            QPushButton:disabled { background-color: #bdc3c7; color: #888; }
        """
        for idx, btn in enumerate(self.actuator_buttons):
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(lambda ch, i=idx: self.toggle(i))
            act_layout.addWidget(btn, idx // 2, idx % 2)

        right_layout.addWidget(act_group)

        # MANUAL INPUT SLIDERS
        slider_group = QGroupBox("Manual Control Parameters")
        slider_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        slider_layout = QVBoxLayout(slider_group)
        slider_layout.setSpacing(4)

        # EXTRUDER FREQUENCY SPEED
        self.lbl_slider = QLabel(f"Extruder Speed: {self.velocidad_extrusor}")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(10, 100)
        self.slider.setValue(self.velocidad_extrusor)
        self.slider.valueChanged.connect(self.actualizar_velocidad_extrusor)
        slider_layout.addWidget(self.lbl_slider)
        slider_layout.addWidget(self.slider)

        # TEMPERATURE
        self.lbl_temp = QLabel(f"Target Temperature: {self.temperatura_objetivo} °C")
        self.slider_temp = QSlider(Qt.Horizontal)
        self.slider_temp.setRange(30, 230)
        self.slider_temp.setValue(self.temperatura_objetivo)
        self.slider_temp.valueChanged.connect(self.actualizar_temperatura)
        slider_layout.addWidget(self.lbl_temp)
        slider_layout.addWidget(self.slider_temp)

        # DC MOTOR SPEED
        self.lbl_dc = QLabel(f"DC Motor Speed (RPM): {self.velocidad_dc_objetivo}")
        self.slider_dc = QSlider(Qt.Horizontal)
        self.slider_dc.setRange(5, 60)
        self.slider_dc.setValue(self.velocidad_dc_objetivo)
        self.slider_dc.valueChanged.connect(self.actualizar_velocidad_dc)
        slider_layout.addWidget(self.lbl_dc)
        slider_layout.addWidget(self.slider_dc)

        # FAN SPEED
        self.lbl_fan_speed = QLabel("Fan Speed (%): 0")
        self.slider_fan = QSlider(Qt.Horizontal)
        self.slider_fan.setRange(0, 100)
        self.slider_fan.setValue(0)
        self.slider_fan.valueChanged.connect(self.actualizar_velocidad_fan)
        slider_layout.addWidget(self.lbl_fan_speed)
        slider_layout.addWidget(self.slider_fan)

        right_layout.addWidget(slider_group)

        self.export_button = QPushButton("📁  Export CSV")
        self.export_button.setStyleSheet(
            "QPushButton { padding: 7px; background-color: #2980b9; color: white; "
            "font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #3498db; }"
        )
        self.export_button.clicked.connect(self.export_csv)
        right_layout.addWidget(self.export_button)

        # PID GAINS IN ARDUINO
        pid_group = QGroupBox("PID Gain Tuning (Arduino)")
        pid_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        pid_layout = QFormLayout(pid_group)
        pid_layout.setSpacing(5)
        pid_layout.setLabelAlignment(Qt.AlignRight)

        self.le_kp_h = QLineEdit("1.8")
        self.le_ki_h = QLineEdit("0.9")
        self.le_kd_h = QLineEdit("0.3")
        pid_layout.addRow("Hotend Kp:", self.le_kp_h)
        pid_layout.addRow("Hotend Ki:", self.le_ki_h)
        pid_layout.addRow("Hotend Kd:", self.le_kd_h)
        pid_layout.addRow(make_separator())

        self.le_kp_m = QLineEdit("25.0")
        self.le_ki_m = QLineEdit("2.5")
        self.le_kd_m = QLineEdit("1.5")
        pid_layout.addRow("Motor Kp:", self.le_kp_m)
        pid_layout.addRow("Motor Ki:", self.le_ki_m)
        pid_layout.addRow("Motor Kd:", self.le_kd_m)

        right_layout.addWidget(pid_group)

        self.btn_update_pids = QPushButton("Update PID K Gains")
        self.btn_update_pids.setStyleSheet(
            "QPushButton { padding: 6px; background-color: #8e44ad; color: white; "
            "font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #9b59b6; }"
        )
        self.btn_update_pids.clicked.connect(self.actualizar_pids)
        right_layout.addWidget(self.btn_update_pids)

        # ── Cascade Diameter Control ──
        diameter_group = QGroupBox("Cascade Diameter Control")
        diameter_group.setStyleSheet(
            "QGroupBox { font-weight: bold; border: 2px solid #e67e22; "
            "border-radius: 6px; margin-top: 8px; padding-top: 6px; }"
            "QGroupBox::title { color: #e67e22; subcontrol-origin: margin; left: 10px; }"
        )
        diameter_layout = QVBoxLayout(diameter_group)
        diameter_layout.setSpacing(8)
        diameter_layout.setContentsMargins(10, 14, 10, 10)

        self.cb_enable_diameter = QCheckBox("Enable Diameter Control")
        self.cb_enable_diameter.setStyleSheet("font-size: 13px; font-weight: bold; color: #e67e22;")
        self.cb_enable_diameter.stateChanged.connect(self.toggle_diameter_control)
        diameter_layout.addWidget(self.cb_enable_diameter)

        diameter_layout.addWidget(make_separator())

        param_grid = QGridLayout()
        param_grid.setSpacing(6)
        param_grid.setColumnMinimumWidth(0, 120)

        def make_diam_label(text):
            lbl = QLabel(text)
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            return lbl

        param_grid.addWidget(make_diam_label("Setpoint (mm):"), 0, 0)
        self.spin_setpoint = QDoubleSpinBox()
        self.spin_setpoint.setRange(1.0, 3.0)
        self.spin_setpoint.setSingleStep(0.05)
        self.spin_setpoint.setValue(1.75)
        self.spin_setpoint.valueChanged.connect(self.update_pid_setpoint)
        param_grid.addWidget(self.spin_setpoint, 0, 1)

        param_grid.addWidget(make_diam_label("Kp:"), 1, 0)
        self.le_kp_diam = QLineEdit("2.0")
        param_grid.addWidget(self.le_kp_diam, 1, 1)

        param_grid.addWidget(make_diam_label("Ki:"), 2, 0)
        self.le_ki_diam = QLineEdit("0.5")
        param_grid.addWidget(self.le_ki_diam, 2, 1)

        param_grid.addWidget(make_diam_label("Kd:"), 3, 0)
        self.le_kd_diam = QLineEdit("0.1")
        param_grid.addWidget(self.le_kd_diam, 3, 1)

        diameter_layout.addLayout(param_grid)

        btn_row = QHBoxLayout()
        self.btn_update_diam_pid = QPushButton("Update PID")
        self.btn_update_diam_pid.setStyleSheet(
            "QPushButton { padding: 6px; background-color: #e67e22; color: white; "
            "font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #f39c12; }"
        )
        self.btn_update_diam_pid.clicked.connect(self.update_master_pid_gains)

        self.btn_reset_pid = QPushButton("Reset PID")
        self.btn_reset_pid.setStyleSheet(
            "QPushButton { padding: 6px; background-color: #7f8c8d; color: white; "
            "font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #95a5a6; }"
        )
        self.btn_reset_pid.clicked.connect(self.reset_master_pid)

        btn_row.addWidget(self.btn_update_diam_pid)
        btn_row.addWidget(self.btn_reset_pid)
        diameter_layout.addLayout(btn_row)

        self.lbl_rpm_setpoint = QLabel("Current RPM Setpoint: --")
        self.lbl_rpm_setpoint.setStyleSheet(
            "font-weight: bold; font-size: 13px; color: #e67e22; "
            "background-color: #fef9f0; border: 1px solid #f0c080; "
            "border-radius: 4px; padding: 4px 8px;"
        )
        self.lbl_rpm_setpoint.setAlignment(Qt.AlignCenter)
        diameter_layout.addWidget(self.lbl_rpm_setpoint)

        right_layout.addWidget(diameter_group)

        # VIDEO
        video_group = QGroupBox("Extrusion Video")
        video_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        video_layout = QVBoxLayout(video_group)
        video_layout.setContentsMargins(4, 8, 4, 4)

        self.label_camara = QLabel("No signal")
        self.label_camara.setFixedSize(380, 285)
        self.label_camara.setAlignment(Qt.AlignCenter)
        self.label_camara.setStyleSheet(
            "border: 2px solid #555; background-color: #111; color: #aaa; font-size: 14px;"
        )
        video_layout.addWidget(self.label_camara, alignment=Qt.AlignCenter)

        right_layout.addWidget(video_group)
        right_layout.addStretch()

        right_scroll.setWidget(right_inner)
        main_layout.addWidget(right_scroll, 2)

    #PID MASTER EDITOR

    def update_pid_setpoint(self):
        self.pid_diameter.setpoint = self.spin_setpoint.value()

    def update_master_pid_gains(self):
        try:
            kp = float(self.le_kp_diam.text())
            ki = float(self.le_ki_diam.text())
            kd = float(self.le_kd_diam.text())
            self.pid_diameter.Kp = kp
            self.pid_diameter.Ki = ki
            self.pid_diameter.Kd = kd
            print(f"Diameter PID gains updated: Kp={kp}, Ki={ki}, Kd={kd}")
        except ValueError:
            print("Error: invalid PID values")

    def reset_master_pid(self):
        self.pid_diameter.reset()
        print("Diameter PID reset")

    # ENABLE AND DISABLE DIAMETER CONTROL

    def toggle_diameter_control(self, state):
        enabled = (state == Qt.Checked)
        self.diameter_control_enabled = enabled

        if enabled:
            self.saved_manual_values = {
                'estado': self.estado.copy(),
                'velocidad_extrusor': self.velocidad_extrusor,
                'temperatura_objetivo': self.temperatura_objetivo,
                'velocidad_dc_objetivo': self.velocidad_dc_objetivo,
                'velocidad_fan': self.velocidad_fan
            }
            self.set_actuator_state(0, '1')
            self.set_actuator_state(1, '0')
            self.set_actuator_state(2, '1')
            self.set_actuator_state(3, '1')

            self.slider.setValue(10)
            self.actualizar_velocidad_extrusor(10)
            self.slider_temp.setValue(220)
            self.actualizar_temperatura(220)
            self.slider_fan.setValue(0)
            self.actualizar_velocidad_fan(0)

            self.slider_dc.setEnabled(False)
            self.set_manual_controls_enabled(False)

            self.pid_diameter.reset()
            self.pid_diameter.setpoint = self.spin_setpoint.value()
            self.last_pid_time = time.monotonic()
            print("Diameter control ENABLED")
        else:
            if self.saved_manual_values:
                saved_estado = self.saved_manual_values['estado']
                for i in range(4):
                    self.set_actuator_state(i, saved_estado[i])

                self.slider.setValue(self.saved_manual_values['velocidad_extrusor'])
                self.actualizar_velocidad_extrusor(self.saved_manual_values['velocidad_extrusor'])
                self.slider_temp.setValue(self.saved_manual_values['temperatura_objetivo'])
                self.actualizar_temperatura(self.saved_manual_values['temperatura_objetivo'])
                self.slider_dc.setValue(self.saved_manual_values['velocidad_dc_objetivo'])
                self.actualizar_velocidad_dc(self.saved_manual_values['velocidad_dc_objetivo'])
                self.slider_fan.setValue(self.saved_manual_values['velocidad_fan'])
                self.actualizar_velocidad_fan(self.saved_manual_values['velocidad_fan'])

            self.slider_dc.setEnabled(True)
            self.set_manual_controls_enabled(True)
            print("Diameter control DISABLED")

    def set_actuator_state(self, index, state):
        if self.estado[index] != state:
            self.estado[index] = state
            btn = self.actuator_buttons[index]
            name = self.actuator_names[index]
            btn.setText(f"{name} ({'ON' if state == '1' else 'OFF'})")
            self.actualizar_indicador_estado(index)

    def set_manual_controls_enabled(self, enabled):
        for btn in self.actuator_buttons:
            btn.setEnabled(enabled)
        self.slider.setEnabled(enabled)
        self.slider_temp.setEnabled(enabled)
        self.slider_fan.setEnabled(enabled)
        if not self.diameter_control_enabled:
            self.slider_dc.setEnabled(enabled)

    # COMPUTER VISION

    def actualizar_imagen_camara(self):
        if not (hasattr(self, 'cap') and self.cap and self.cap.isOpened()):
            return
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gris, (5, 5), 0)
        _, binaria = cv2.threshold(blur, self.VALOR_UMBRAL, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binaria = cv2.morphologyEx(binaria, cv2.MORPH_OPEN, kernel)
        binaria = cv2.morphologyEx(binaria, cv2.MORPH_CLOSE, kernel)

        contornos, _ = cv2.findContours(binaria, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contornos:
            main_contorno = max(contornos, key=cv2.contourArea)
            if cv2.contourArea(main_contorno) < 100:
                vis = cv2.resize(frame, (380, 285))
                h, w, ch = vis.shape
                qimg = QImage(vis.data, w, h, ch * w, QImage.Format_RGB888)
                self.label_camara.setPixmap(QPixmap.fromImage(qimg))
                return

            hull = cv2.convexHull(main_contorno)
            rect = cv2.minAreaRect(hull)
            cen, tam, ang = rect
            if tam[0] > tam[1]:
                tam = (tam[1], tam[0])
                ang = ang + 90

            M = cv2.getRotationMatrix2D(cen, ang, 1)
            puntos = hull.reshape(-1, 2)
            puntos_homog = np.hstack([puntos, np.ones((puntos.shape[0], 1))])
            puntos_rotados = M.dot(puntos_homog.T).T

            centro_x = cen[0]
            p_izq = puntos_rotados[puntos_rotados[:, 0] < centro_x]
            p_der = puntos_rotados[puntos_rotados[:, 0] >= centro_x]

            if len(p_izq) >= 5 and len(p_der) >= 5:
                y_min = max(p_izq[:, 1].min(), p_der[:, 1].min())
                y_max = min(p_izq[:, 1].max(), p_der[:, 1].max())
                if y_max - y_min >= 10:
                    num_niveles = 20
                    niveles_y = np.linspace(y_min, y_max, num_niveles)
                    anchos = []
                    for i in range(len(niveles_y) - 1):
                        y_inf, y_sup = niveles_y[i], niveles_y[i + 1]
                        mask_izq = (p_izq[:, 1] >= y_inf) & (p_izq[:, 1] < y_sup)
                        mask_der = (p_der[:, 1] >= y_inf) & (p_der[:, 1] < y_sup)
                        if np.any(mask_izq) and np.any(mask_der):
                            ancho_px = np.mean(p_der[mask_der, 0]) - np.mean(p_izq[mask_izq, 0])
                            if ancho_px > 0:
                                anchos.append(ancho_px)

                    if anchos:
                        anchos = np.array(anchos)
                        q1, q3 = np.percentile(anchos, [25, 75])
                        iqr = q3 - q1
                        filtro = (anchos > q1 - 1.5 * iqr) & (anchos < q3 + 1.5 * iqr)
                        anchos_filtrados = anchos[filtro] if np.any(filtro) else anchos
                        ancho_px_medio = np.mean(anchos_filtrados)
                        dist_mm = ancho_px_medio * self.FACTOR_CONVERSION
                        self.grosor_data.append(dist_mm)
                        self.label_grosor_display.setText(f"{dist_mm:.3f} mm")

                        box = np.array(cv2.boxPoints(rect), dtype=np.int32)
                        cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                        cv2.drawContours(frame, [main_contorno], -1, (255, 0, 0), 1)
                        cv2.drawContours(frame, [hull], -1, (0, 0, 255), 1)
                        cv2.putText(frame, f"Diameter: {dist_mm:.3f} mm", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        vis = cv2.resize(frame, (380, 285))
        h, w, ch = vis.shape
        qimg = QImage(vis.data, w, h, ch * w, QImage.Format_RGB888)
        self.label_camara.setPixmap(QPixmap.fromImage(qimg))

    # ----------------------------------------------------------------
    # Serial communication
    # ----------------------------------------------------------------
    def actualizar_pids(self):
        try:
            cmd_pid_h = f"PIDH:{self.le_kp_h.text()},{self.le_ki_h.text()},{self.le_kd_h.text()}\n"
            arduino.write(cmd_pid_h.encode())
            cmd_pid_m = f"PIDM:{self.le_kp_m.text()},{self.le_ki_m.text()},{self.le_kd_m.text()}\n"
            arduino.write(cmd_pid_m.encode())
            print(f"PIDs sent: {cmd_pid_h.strip()} | {cmd_pid_m.strip()}")
        except Exception as e:
            print(f"Error sending PID gains: {e}")

    def toggle(self, index):
        if not self.diameter_control_enabled:
            self.estado[index] = '1' if self.estado[index] == '0' else '0'
            self.actualizar_indicador_estado(index)
            name = self.actuator_names[index]
            state = "ON" if self.estado[index] == '1' else "OFF"
            self.actuator_buttons[index].setText(f"{name} ({state})")

    def actualizar_indicador_estado(self, index):
        dot = self.indicator_colors[index]
        lbl = self.indicator_labels[index]
        name = self.actuator_names[index]
        if self.estado[index] == '1':
            dot.setStyleSheet("background-color: #27ae60; border-radius: 9px;")
            lbl.setText(f"{name}: ON")
        else:
            dot.setStyleSheet("background-color: #e74c3c; border-radius: 9px;")
            lbl.setText(f"{name}: OFF")

    def actualizar_velocidad_extrusor(self, val):
        self.velocidad_extrusor = val
        self.lbl_slider.setText(f"Extruder Speed: {val}")

    def actualizar_temperatura(self, val):
        self.temperatura_objetivo = val
        self.lbl_temp.setText(f"Target Temperature: {val} °C")

    def actualizar_velocidad_dc(self, val):
        self.velocidad_dc_objetivo = val
        self.lbl_dc.setText(f"DC Motor Speed (RPM): {val}")

    def actualizar_velocidad_fan(self, val):
        self.velocidad_fan = val
        self.lbl_fan_speed.setText(f"Fan Speed (%): {val}")

    def actualizar(self):
        try:
            arduino.write(f"ACTUATE:{''.join(self.estado)}\n".encode())

            if self.diameter_control_enabled:
                arduino.write(f"SPEED:{self.velocidad_extrusor}\n".encode())
                arduino.write(f"TEMP:{self.temperatura_objetivo}\n".encode())
                arduino.write(f"FANSPEED:{self.velocidad_fan}\n".encode())

                if len(self.grosor_data) > 0:
                    current_diameter = self.grosor_data[-1]
                    now = time.monotonic()
                    dt = now - self.last_pid_time
                    if dt > 0.01:
                        rpm_setpoint = self.pid_diameter.update(current_diameter, dt)
                        rpm_setpoint = max(10, min(30, rpm_setpoint))
                        self.last_rpm_setpoint = rpm_setpoint
                        self.last_pid_time = now
                        arduino.write(f"DCSPEED:{rpm_setpoint:.1f}\n".encode())
                        self.lbl_rpm_setpoint.setText(f"Current RPM Setpoint: {rpm_setpoint:.1f}")
                        self.slider_dc.setValue(int(rpm_setpoint))
                else:
                    arduino.write(f"DCSPEED:{self.last_rpm_setpoint:.1f}\n".encode())
                    self.lbl_rpm_setpoint.setText(f"Current RPM Setpoint: {self.last_rpm_setpoint:.1f} (no measurement)")
            else:
                arduino.write(f"SPEED:{self.velocidad_extrusor}\n".encode())
                arduino.write(f"TEMP:{self.temperatura_objetivo}\n".encode())
                arduino.write(f"DCSPEED:{self.velocidad_dc_objetivo}\n".encode())
                arduino.write(f"FANSPEED:{self.velocidad_fan}\n".encode())
        except Exception as e:
            print(f"Error sending commands: {e}")

        try:
            while arduino.in_waiting:
                try:
                    line = arduino.readline().decode(errors='ignore').strip()
                    if line.startswith("Temp:"):
                        self.temp_data.append(float(line.split(':', 1)[1]))
                    elif line.startswith("Motor DC RPM:"):
                        rpm = float(line.split(":", 1)[1])
                        self.motor_rpm_data.append(rpm)
                        self.motor_dc_state_data.append(1 if rpm > 0 else 0)
                    elif line.startswith("Fan:"):
                        self.fan_state_data.append(1 if "Encendido" in line else 0)
                    elif line.startswith("Extruder:"):
                        self.extruder_state_data.append(1 if "Encendido" in line else 0)
                    elif line.startswith("Heater:"):
                        self.heater_state_data.append(1 if "Encendido" in line else 0)
                except Exception:
                    break
        except Exception:
            pass

        max_len = 100
        self.canvas_temp.plot(self.temp_data[-max_len:], ylabel="Temp (°C)")
        self.canvas_motor.plot(self.motor_rpm_data[-max_len:], ylabel="Motor DC (RPM)")
        self.canvas_grosor.plot(self.grosor_data[-max_len:], ylabel="Diameter (mm)")

    def export_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "session_data.csv", "CSV Files (*.csv)")
        if not path:
            return
        all_data = [
            self.temp_data, self.motor_rpm_data, self.grosor_data,
            self.motor_dc_state_data, self.fan_state_data,
            self.extruder_state_data, self.heater_state_data
        ]
        max_rows = max((len(d) for d in all_data), default=0)
        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'Timestamp', 'Hotend_Temperature', 'DC_Motor_RPM', 'Filament_Diameter',
                    'DC_Motor_State', 'Fan_State', 'Extruder_State', 'Heater_State'
                ])
                for i in range(max_rows):
                    writer.writerow([
                        i,
                        self.temp_data[i] if i < len(self.temp_data) else '',
                        self.motor_rpm_data[i] if i < len(self.motor_rpm_data) else '',
                        self.grosor_data[i] if i < len(self.grosor_data) else '',
                        self.motor_dc_state_data[i] if i < len(self.motor_dc_state_data) else '',
                        self.fan_state_data[i] if i < len(self.fan_state_data) else '',
                        self.extruder_state_data[i] if i < len(self.extruder_state_data) else '',
                        self.heater_state_data[i] if i < len(self.heater_state_data) else ''
                    ])
            print(f"Data saved to {path}")
        except Exception as e:
            print(f"Error saving CSV: {e}")

    def closeEvent(self, event):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = ControlGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
