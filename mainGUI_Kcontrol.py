import sys
import platform
import serial
import serial.tools.list_ports
import numpy as np
import csv
import cv2

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QFileDialog, QLineEdit, QFormLayout
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# -----------------------
# Serial: auto-detect puerto
# -----------------------
def encontrar_puerto_arduino():
    puertos = serial.tools.list_ports.comports()
    for p in puertos:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if ("arduino" in desc) or ("usb serial" in desc) or ("ch340" in desc) or ("1a86" in hwid) or ("2341" in hwid):
            return p.device

    if platform.system() == "Windows": return "COM14"
    elif platform.system() == "Linux": return "/dev/ttyACM0"
    elif platform.system() == "Darwin": return "/dev/tty.usbmodem1101"
    else: return None

try:
    puerto = encontrar_puerto_arduino() or "/dev/ttyACM0"
    arduino = serial.Serial(puerto, 115200, timeout=1)
except Exception as e:
    print(f"Error al abrir el puerto serial: {e}")
    sys.exit(1)

# -----------------------
# PlotCanvas: contenedor de gráficas
# -----------------------
class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=2, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)
        self.setParent(parent)

    def plot(self, data, ylabel=""):
        self.axes.cla()
        if len(data) > 0: self.axes.plot(data, linestyle='-')
        if ylabel: self.axes.set_ylabel(ylabel)
        self.axes.grid(True)
        self.draw()

# -----------------------
# GUI principal
# -----------------------
class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AI-FrED0 Control Interface")
        self.resize(1200, 780)

        self.estado = ['0','0','0','0'] # [Motor DC, Fan, Extrusor, Heater]
        self.velocidad_extrusor = 100
        self.temperatura_objetivo = 190
        self.velocidad_dc_objetivo = 20
        self.velocidad_fan = 0

        
        # --- Listas para almacenar todos los datos históricos ---
        self.temp_data = []
        self.motor_rpm_data = []
        self.grosor_data = []
        self.motor_dc_state_data = []
        self.fan_state_data = []
        self.extruder_state_data = []
        self.heater_state_data = []
        
        self.FACTOR_CONVERSION, self.VALOR_UMBRAL = 0.25 / 43, 127
        
        self.actuator_names = ["Motor DC", "Fan", "Extrusor", "Heater"]

        main_layout = QHBoxLayout(self)
        
        # --- Panel de gráficas e indicadores (izquierda) ---
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        self.canvas_temp = PlotCanvas(self, width=5, height=2)
        self.canvas_motor = PlotCanvas(self, width=5, height=2)
        self.canvas_grosor = PlotCanvas(self, width=5, height=2)
        left_layout.addWidget(self.canvas_temp)
        left_layout.addWidget(self.canvas_motor)
        left_layout.addWidget(self.canvas_grosor)

        bottom_left_container = QWidget()
        bottom_left_layout = QHBoxLayout(bottom_left_container)
        
        status_container = QWidget()
        status_layout = QVBoxLayout(status_container)
        status_layout.setContentsMargins(10, 20, 10, 10)
        
        status_label = QLabel("Estado de Actuadores")
        status_label.setStyleSheet("font-weight: bold; font-size: 14px; margin-bottom: 10px;")
        status_layout.addWidget(status_label)

        self.indicator_colors = []
        self.indicator_labels = []

        for name in self.actuator_names:
            indicator_row_layout = QHBoxLayout()
            color_label = QLabel()
            color_label.setFixedSize(20, 20)
            self.indicator_colors.append(color_label)
            text_label = QLabel(f"{name}: APAGADO")
            text_label.setStyleSheet("font-size: 12px;")
            self.indicator_labels.append(text_label)
            indicator_row_layout.addWidget(color_label)
            indicator_row_layout.addWidget(text_label)
            indicator_row_layout.addStretch()
            status_layout.addLayout(indicator_row_layout)
        
        bottom_left_layout.addWidget(status_container)

        grosor_display_container = QWidget()
        grosor_display_layout = QVBoxLayout(grosor_display_container)
        grosor_display_layout.setContentsMargins(40, 20, 10, 10)

        self.label_grosor_display = QLabel("GROSOR DEL FILAMENTO:\n-- mm")
        self.label_grosor_display.setStyleSheet("font-weight: bold; font-size: 18px; color: #3498db;")
        self.label_grosor_display.setAlignment(Qt.AlignCenter)
        grosor_display_layout.addWidget(self.label_grosor_display)
        
        bottom_left_layout.addWidget(grosor_display_container)
        
        left_layout.addWidget(bottom_left_container)
        left_layout.addStretch()
        
        main_layout.addWidget(left_widget, 3)

        # --- Panel de controles (derecha) ---
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)

        self.btn_spool = QPushButton(f"{self.actuator_names[0]} (OFF)")
        self.btn_fan = QPushButton(f"{self.actuator_names[1]} (OFF)")
        self.btn_extrude = QPushButton(f"{self.actuator_names[2]} (OFF)")
        self.btn_heater = QPushButton(f"{self.actuator_names[3]} (OFF)")
        
        self.actuator_buttons = [self.btn_spool, self.btn_fan, self.btn_extrude, self.btn_heater]
        
        for idx, btn in enumerate(self.actuator_buttons):
            btn.clicked.connect(lambda ch, i=idx: self.toggle(i))
            right_layout.addWidget(btn)

        self.lbl_slider = QLabel(f"Velocidad Extrusor: {self.velocidad_extrusor}")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(10, 100)
        self.slider.setValue(self.velocidad_extrusor)
        self.slider.valueChanged.connect(self.actualizar_velocidad_extrusor)
        right_layout.addWidget(self.lbl_slider)
        right_layout.addWidget(self.slider)

        self.lbl_temp = QLabel(f"Temperatura objetivo: {self.temperatura_objetivo} °C")
        self.slider_temp = QSlider(Qt.Horizontal)
        self.slider_temp.setRange(30, 230)
        self.slider_temp.setValue(self.temperatura_objetivo)
        self.slider_temp.valueChanged.connect(self.actualizar_temperatura)
        right_layout.addWidget(self.lbl_temp)
        right_layout.addWidget(self.slider_temp)

        self.lbl_dc = QLabel(f"Velocidad Motor DC (RPM): {self.velocidad_dc_objetivo}")
        self.slider_dc = QSlider(Qt.Horizontal)
        self.slider_dc.setRange(5, 60)
        self.slider_dc.setValue(self.velocidad_dc_objetivo)
        self.slider_dc.valueChanged.connect(self.actualizar_velocidad_dc)
        right_layout.addWidget(self.lbl_dc)
        right_layout.addWidget(self.slider_dc)

        self.lbl_fan_speed = QLabel("Velocidad del Ventilador (%): 0")
        self.slider_fan = QSlider(Qt.Horizontal)
        self.slider_fan.setRange(0, 100)
        self.slider_fan.setValue(0)
        self.slider_fan.valueChanged.connect(self.actualizar_velocidad_fan)
        right_layout.addWidget(self.lbl_fan_speed)
        right_layout.addWidget(self.slider_fan)


        self.export_button = QPushButton("Exportar CSV")
        self.export_button.clicked.connect(self.export_csv)
        right_layout.addWidget(self.export_button)

        right_layout.addStretch()
        pid_label = QLabel("Ajuste de Ganancias K PID")
        pid_label.setStyleSheet("font-weight: bold; margin-top: 10px;")
        right_layout.addWidget(pid_label)
        pid_form_layout = QFormLayout()

        self.le_kp_h = QLineEdit("25.0")
        self.le_ki_h = QLineEdit("2.5")
        self.le_kd_h = QLineEdit("1.5")
        pid_form_layout.addRow("Hotend Kp:", self.le_kp_h)
        pid_form_layout.addRow("Hotend Ki:", self.le_ki_h)
        pid_form_layout.addRow("Hotend Kd:", self.le_kd_h)
        
        self.le_kp_m = QLineEdit("1.8")
        self.le_ki_m = QLineEdit("0.9")
        self.le_kd_m = QLineEdit("0.3")
        pid_form_layout.addRow("Motor Kp:", self.le_kp_m)
        pid_form_layout.addRow("Motor Ki:", self.le_ki_m)
        pid_form_layout.addRow("Motor Kd:", self.le_kd_m)
        
        right_layout.addLayout(pid_form_layout)

        self.btn_update_pids = QPushButton("Actualizar Ganancias K PID")
        self.btn_update_pids.clicked.connect(self.actualizar_pids)
        right_layout.addWidget(self.btn_update_pids)
        right_layout.addStretch()
        
        self.label_video = QLabel("Video de Extrusión")
        self.label_video.setStyleSheet("font-weight: bold;")
        right_layout.addWidget(self.label_video)
        self.label_camara = QLabel("Sin señal")
        self.label_camara.setFixedSize(420, 315)
        self.label_camara.setAlignment(Qt.AlignCenter)
        self.label_camara.setStyleSheet("border: 2px solid gray; background-color: #111; color: white;")
        right_layout.addWidget(self.label_camara)

        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) if platform.system() == "Windows" else cv2.VideoCapture(0)
        if not self.cap.isOpened(): self.label_camara.setText("Cámara no disponible")

        self.timer_camara = QTimer()
        self.timer_camara.timeout.connect(self.actualizar_imagen_camara)
        self.timer_camara.start(500)
        main_layout.addWidget(right_widget, 1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.actualizar)
        self.timer.start(250)

        for i in range(4):
            self.actualizar_indicador_estado(i)

    def closeEvent(self, event):
        if hasattr(self, 'cap') and self.cap.isOpened(): self.cap.release()
        cv2.destroyAllWindows()
        event.accept()

    def actualizar_imagen_camara(self):
        if not (hasattr(self, 'cap') and self.cap.isOpened()): return
        ret, frame = self.cap.read()
        if not ret or frame is None: return

        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gris, (5, 5), 0)
        _, umbral = cv2.threshold(blur, self.VALOR_UMBRAL, 255, cv2.THRESH_BINARY)
        frame_resultado = cv2.cvtColor(umbral, cv2.COLOR_GRAY2BGR)

        contornos, _ = cv2.findContours(umbral, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contornos:
            contorno_principal = max(contornos, key=cv2.contourArea)
            if cv2.contourArea(contorno_principal) > 100:
                puntos = contorno_principal.squeeze()
                if puntos.ndim != 1 and len(puntos) >= 10:
                    centro_x = frame.shape[1] / 2
                    p_izq = puntos[puntos[:, 0] < centro_x]
                    p_der = puntos[puntos[:, 0] >= centro_x]

                    if len(p_izq) > 0 and len(p_der) > 0:
                        distancias_px = [np.min(np.linalg.norm(p_der - p, axis=1)) for p in p_izq[::5]]
                        if distancias_px:
                            dist_mm = np.mean(distancias_px) * self.FACTOR_CONVERSION
                            texto = f"Grosor: {dist_mm:.3f} mm"
                            cv2.putText(frame_resultado, texto, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
                            
                            self.grosor_data.append(dist_mm)
                            self.label_grosor_display.setText(f"GROSOR DEL FILAMENTO:\n{dist_mm:.3f} mm")
        
        vis = cv2.resize(frame_resultado, (420, 315))
        h, w, ch = vis.shape
        qimg = QImage(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB).data, w, h, ch * w, QImage.Format_RGB888)
        self.label_camara.setPixmap(QPixmap.fromImage(qimg))

    def actualizar_pids(self):
        try:
            kp_h, ki_h, kd_h = self.le_kp_h.text(), self.le_ki_h.text(), self.le_kd_h.text()
            cmd_pid_h = f"PIDH:{kp_h},{ki_h},{kd_h}\n"
            arduino.write(cmd_pid_h.encode())
            print(f"Enviando comando Hotend: {cmd_pid_h.strip()}")

            kp_m, ki_m, kd_m = self.le_kp_m.text(), self.le_ki_m.text(), self.le_kd_m.text()
            cmd_pid_m = f"PIDM:{kp_m},{ki_m},{kd_m}\n"
            arduino.write(cmd_pid_m.encode())
            print(f"Enviando comando Motor: {cmd_pid_m.strip()}")
        except Exception as e:
            print(f"Error al enviar ganancias PID: {e}")

    def toggle(self, index):
        self.estado[index] = '1' if self.estado[index] == '0' else '0'
        print("Nuevo estado:", self.estado)  
        self.actualizar_indicador_estado(index)

        button = self.actuator_buttons[index]
        button_name = self.actuator_names[index]

        if self.estado[index] == '1':
            button.setText(f"{button_name} (ON)")
        else:
            button.setText(f"{button_name} (OFF)")

    def actualizar_indicador_estado(self, index):
        color_label = self.indicator_colors[index]
        text_label = self.indicator_labels[index]
        actuator_name = self.actuator_names[index] 

        if self.estado[index] == '1':
            color_label.setStyleSheet("background-color: #4CAF50; border-radius: 10px;")
            text_label.setText(f"{actuator_name}: ENCENDIDO")
        else:
            color_label.setStyleSheet("background-color: #F44336; border-radius: 10px;")
            text_label.setText(f"{actuator_name}: APAGADO")

    def actualizar_velocidad_extrusor(self, val):
        self.velocidad_extrusor = val
        self.lbl_slider.setText(f"Velocidad Extrusor: {val}")

    def actualizar_temperatura(self, val):
        self.temperatura_objetivo = val
        self.lbl_temp.setText(f"Temperatura objetivo: {val} °C")

    def actualizar_velocidad_dc(self, val):
        self.velocidad_dc_objetivo = val
        self.lbl_dc.setText(f"Velocidad Motor DC (RPM): {val}")
    
    def actualizar_velocidad_fan(self, val):
        self.velocidad_fan = val
        self.lbl_fan_speed.setText(f"Velocidad del Ventilador (%): {val}")


    def actualizar(self):
        try:
            arduino.write(f"ACTUATE:{''.join(self.estado)}\n".encode())
            arduino.write(f"SPEED:{self.velocidad_extrusor}\n".encode())
            arduino.write(f"TEMP:{self.temperatura_objetivo}\n".encode())
            arduino.write(f"DCSPEED:{self.velocidad_dc_objetivo}\n".encode())
            arduino.write(f"FANSPEED:{self.velocidad_fan}\n".encode())

        except Exception: pass

        try:
            while arduino.in_waiting:
                try:
                    line = arduino.readline().decode(errors='ignore').strip()
                    if line.startswith("Temp:"): 
                        self.temp_data.append(float(line.split(':',1)[1]))
                    elif line.startswith("Motor DC RPM:"):
                        rpm = float(line.split(":",1)[1])
                        self.motor_rpm_data.append(rpm)
                        self.motor_dc_state_data.append(1 if rpm > 0 else 0)
                    elif line.startswith("Fan:"): 
                        self.fan_state_data.append(1 if "Encendido" in line else 0)
                    elif line.startswith("Extruder:"): 
                        self.extruder_state_data.append(1 if "Encendido" in line else 0)
                    elif line.startswith("Heater:"):
                        self.heater_state_data.append(1 if "Encendido" in line else 0)
                except Exception: break
        except Exception: pass

        # ##################################################################
        # # INICIO: CAMBIO PARA EXPORTAR CSV COMPLETO                    #
        # ##################################################################
        
        # Define el número de puntos a mostrar en la gráfica
        max_len_grafica = 100
        
        # Pasa solo los últimos 'max_len_grafica' puntos a la función de ploteo
        # Las listas principales (self.temp_data, etc.) ya no se recortan.
        self.canvas_temp.plot(self.temp_data[-max_len_grafica:], ylabel="Temp (°C)")
        self.canvas_motor.plot(self.motor_rpm_data[-max_len_grafica:], ylabel="Motor DC (RPM)")
        self.canvas_grosor.plot(self.grosor_data[-max_len_grafica:], ylabel="Grosor (mm)")
        
        # ##################################################################
        # # FIN: CAMBIO PARA EXPORTAR CSV COMPLETO                       #
        # ##################################################################

    def export_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, "Guardar CSV", "datos_sesion.csv", "CSV Files (*.csv)")
        if not path:
            return

        all_data = [
            self.temp_data, self.motor_rpm_data, self.grosor_data,
            self.motor_dc_state_data, self.fan_state_data, 
            self.extruder_state_data, self.heater_state_data
        ]
        max_rows = max(len(d) for d in all_data) if all_data else 0

        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'Timestamp', 'Temperatura_Hotend', 'RPM_Motor_DC', 'Grosor_Filamento',
                    'Estado_Motor_DC', 'Estado_Fan', 'Estado_Extrusor', 'Estado_Heater'
                ])
                
                for i in range(max_rows):
                    row = [
                        i,
                        self.temp_data[i] if i < len(self.temp_data) else '',
                        self.motor_rpm_data[i] if i < len(self.motor_rpm_data) else '',
                        self.grosor_data[i] if i < len(self.grosor_data) else '',
                        self.motor_dc_state_data[i] if i < len(self.motor_dc_state_data) else '',
                        self.fan_state_data[i] if i < len(self.fan_state_data) else '',
                        self.extruder_state_data[i] if i < len(self.extruder_state_data) else '',
                        self.heater_state_data[i] if i < len(self.heater_state_data) else ''
                    ]
                    writer.writerow(row)
            print(f"Datos guardados exitosamente en {path}")
        except Exception as e:
            print(f"Error al guardar el archivo CSV: {e}")

def main():
    app = QApplication(sys.argv)
    gui = ControlGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()