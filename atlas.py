# Taller Segundo Corte - Punto C (PC)
# Atlas (Boston Dynamics) con control articular y camaras RGB / Depth / Segmentacion.
# Uso: python atlas_C_pots_esp32.py --puerto COM6
import argparse
import math
import os
import threading
import time

import numpy as np
import pybullet as p
import pybullet_data
import serial

DT = 1.0 / 240.0
K_SEGUIMIENTO = 4.0
W_MAX = 1.0
ACC_MAX = 3.0
PASO_CAM = 8
CAM_W, CAM_H = 320, 240
LADOS = ("l", "r")
NOMBRE_LADO = {"l": "IZQUIERDO", "r": "DERECHO"}

GRUPOS = [["{s}_arm_shz", "{s}_arm_shx", "{s}_arm_elx"],
          ["{s}_arm_ely", "{s}_arm_wry", "{s}_arm_wrx"],
          ["{s}_leg_hpy", "{s}_leg_kny", "{s}_leg_aky"],
          ["back_bkz", "back_bky", "neck_ry"]]
NOMBRES_GRUPO = ["Hombro", "Brazo y munieca", "Pierna", "Torso y cabeza"]


class LectorESP32(threading.Thread):
    def __init__(self, puerto, baud=115200):
        super().__init__(daemon=True)
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = puerto, baud, 0.1
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        time.sleep(1.0)
        self.ser.reset_input_buffer()
        self.lock = threading.Lock()
        self.dato, self.t = None, 0.0
        self.start()

    def _lineas(self):
        buf = b""
        while True:
            try:
                buf += self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError):
                print("Se perdio la conexion con la ESP32")
                return
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                yield linea.decode("ascii", errors="ignore").strip()
            if len(buf) > 200:
                buf = b""

    def run(self):
        for linea in self._lineas():
            d = linea.split(",")
            if len(d) != 6 or d[0] != "C3":
                continue
            try:
                lado, grupo = int(d[1]) % 2, int(d[2]) % len(GRUPOS)
                pots = np.clip([float(v) for v in d[3:6]], 0.0, 1.0)
            except ValueError:
                continue
            with self.lock:
                self.dato, self.t = (lado, grupo, pots), time.time()

    def leer(self):
        with self.lock:
            if self.dato is None or time.time() - self.t > 1.5:
                return None
            return self.dato


def crear_escena():
    datos = pybullet_data.getDataPath()
    p.loadURDF(os.path.join(datos, "plane.urdf"), [0, 0, 0], useFixedBase=True)
    def cargar(nombre, pos, orn=(0, 0, 0, 1), **kw):
        try:
            return p.loadURDF(os.path.join(datos, nombre), pos, orn, **kw)
        except p.error:
            print("Aviso: no se pudo cargar", nombre)
            return None

    cargar("table/table.urdf", [1.3, 0, 0], p.getQuaternionFromEuler([0, 0, math.pi / 2]),
           useFixedBase=True)
    cargar("tray/traybox.urdf", [1.3, 0.3, 0.63])
    cargar("duck_vhacd.urdf", [1.2, -0.25, 0.7], globalScaling=1.5)
    for pos, color in (([1.35, -0.05, 0.66], [0.9, 0.1, 0.1, 1]),
                       ([1.25, 0.1, 0.66], [0.1, 0.3, 0.9, 1])):
        cubo = cargar("cube_small.urdf", pos)
        if cubo is not None:
            p.changeVisualShape(cubo, -1, rgbaColor=color)

    pts = [[1.8 + 0.8 * math.cos(t), 1.2 * math.sin(t), 0.005]
           for t in np.linspace(math.pi / 2, 3 * math.pi / 2, 30)]
    for a, b in zip(pts[:-1], pts[1:]):
        p.addUserDebugLine(a, b, [1, 0.5, 0], 3)


def buscar_atlas():
    aqui = os.path.dirname(os.path.abspath(__file__))
    carpetas = [pybullet_data.getDataPath(), os.path.join(aqui, "pybullet_robots"),
                aqui, os.path.dirname(aqui)]
    preferidos = ("atlas_v4_with_multisense.urdf", "atlas_v5.urdf", "atlas.urdf")
    encontrados = []
    for carpeta in carpetas:
        if not os.path.isdir(carpeta):
            continue
        for raiz, _, archivos in os.walk(carpeta):
            for a in archivos:
                if a.lower().endswith(".urdf") and "atlas" in a.lower():
                    encontrados.append(os.path.join(raiz, a))
        if encontrados:
            break
    if not encontrados:
        raise FileNotFoundError(
            "No encontre ninguna URDF del Atlas. Ejecuta: python -m pip install -U pybullet")
    encontrados.sort(key=lambda r: (os.path.basename(r) not in preferidos, len(r)))
    print("Atlas encontrado en:", encontrados[0])
    return encontrados[0]


class Atlas:
    def __init__(self):
        urdf = buscar_atlas()
        self.id = p.loadURDF(urdf, [0, 0, 1.0], useFixedBase=True)
        self._apoyar_en_piso()

        self.jnt, self.lnk = {}, {}
        self.movibles, self.ll, self.ul, self.fmax = [], {}, {}, {}
        for j in range(p.getNumJoints(self.id)):
            info = p.getJointInfo(self.id, j)
            self.jnt[info[1].decode()] = j
            self.lnk[info[12].decode()] = j
            if info[2] != p.JOINT_FIXED:
                self.movibles.append(j)
                lo, hi = info[8], info[9]
                if lo >= hi:
                    lo, hi = -math.pi, math.pi
                self.ll[j], self.ul[j] = lo, hi
                self.fmax[j] = info[10] if info[10] > 0 else 200.0
        print("Articulaciones del Atlas:", ", ".join(
            n for n, j in self.jnt.items() if j in self.movibles))
        faltan = [g.format(s=s) for grupo in GRUPOS for g in grupo for s in LADOS
                  if g.format(s=s) not in self.jnt]
        if faltan:
            print("AVISO: estas articulaciones no existen en la URDF y se ignoran:",
                  sorted(set(faltan)))

        self.cabeza = next((j for n, j in self.lnk.items() if n == "head"),
                           next((j for n, j in self.lnk.items() if "head" in n), -1))
        for j in self.movibles:
            p.resetJointState(self.id, j, 0.0)
        self.q_obj = {j: 0.0 for j in self.movibles}
        self.q_cmd = dict(self.q_obj)
        self.w_cmd = {j: 0.0 for j in self.movibles}

    def _apoyar_en_piso(self):
        z_min = min(p.getAABB(self.id, k)[0][2] for k in range(-1, p.getNumJoints(self.id)))
        pos, orn = p.getBasePositionAndOrientation(self.id)
        p.resetBasePositionAndOrientation(self.id, [pos[0], pos[1], pos[2] - z_min + 0.01], orn)

    def fijar_por_pot(self, nombre, valor):
        j = self.jnt.get(nombre)
        if j is None:
            return
        lo, hi = self.ll[j], self.ul[j]
        if lo < 0.0 < hi:
            q = lo * (1.0 - 2.0 * valor) if valor < 0.5 else hi * (2.0 * valor - 1.0)
        else:
            q = lo + (hi - lo) * valor
        self.q_obj[j] = float(q)

    def mover(self, dt):
        dw = ACC_MAX * dt
        for j in self.movibles:
            w_des = float(np.clip(K_SEGUIMIENTO * (self.q_obj[j] - self.q_cmd[j]), -W_MAX, W_MAX))
            self.w_cmd[j] += float(np.clip(w_des - self.w_cmd[j], -dw, dw))
            self.q_cmd[j] += self.w_cmd[j] * dt
        p.setJointMotorControlArray(self.id, self.movibles, p.POSITION_CONTROL,
                                    targetPositions=[self.q_cmd[j] for j in self.movibles],
                                    forces=[self.fmax[j] for j in self.movibles])


PROYECCION = None


def render_camara(atlas):
    global PROYECCION
    if PROYECCION is None:
        PROYECCION = p.computeProjectionMatrixFOV(70, CAM_W / CAM_H, 0.05, 8.0)
    if atlas.cabeza >= 0:
        ls = p.getLinkState(atlas.id, atlas.cabeza, computeForwardKinematics=True)
        pos, orn = np.array(ls[4]), ls[5]
    else:
        pos, orn = p.getBasePositionAndOrientation(atlas.id)
        pos = np.array(pos) + [0, 0, 0.7]
    R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    adelante, arriba = R[:, 0], R[:, 2]
    ojo = pos + adelante * 0.15 + arriba * 0.05
    vista = p.computeViewMatrix(ojo.tolist(), (ojo + adelante).tolist(), arriba.tolist())
    p.getCameraImage(CAM_W, CAM_H, vista, PROYECCION, renderer=p.ER_BULLET_HARDWARE_OPENGL)


def main():
    ap = argparse.ArgumentParser(description="Punto C: Atlas articular con consola ESP32")
    ap.add_argument("--puerto", required=True)
    args = ap.parse_args()

    esp = LectorESP32(args.puerto)

    p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(DT)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    crear_escena()
    atlas = Atlas()
    for cov in (p.COV_ENABLE_GUI, p.COV_ENABLE_RGB_BUFFER_PREVIEW,
                p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW):
        p.configureDebugVisualizer(cov, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.resetDebugVisualizerCamera(3.0, 60, -20, [0.3, 0, 0.9])

    hud = p.addUserDebugText(" ", [0, -1.2, 2.3], [0, 0, 0], textSize=1.2)
    hud2 = p.addUserDebugText(" ", [0, -1.2, 2.15], [0.2, 0.2, 0.6], textSize=1.1)
    paso, t0 = 0, time.time()
    try:
        while p.isConnected():
            dato = esp.leer()
            if dato is not None:
                lado_i, grupo, pots = dato
                s = LADOS[lado_i]
                for plantilla, valor in zip(GRUPOS[grupo], pots):
                    atlas.fijar_por_pot(plantilla.format(s=s), valor)

            atlas.mover(DT)
            p.stepSimulation()

            if paso % PASO_CAM == 0:
                render_camara(atlas)
            if paso % 60 == 0:
                if dato is None:
                    texto, arts = "SIN SENAL ESP32", " "
                else:
                    nombres = [g.format(s=s) for g in GRUPOS[grupo]]
                    texto = (f"Lado {NOMBRE_LADO[s]} | Grupo {grupo + 1}: "
                             f"{NOMBRES_GRUPO[grupo]}")
                    arts = "   ".join(
                        f"{n}={math.degrees(atlas.q_cmd[atlas.jnt[n]]):+.0f} deg"
                        for n in nombres if n in atlas.jnt)
                hud = p.addUserDebugText(texto, [0, -1.2, 2.3], [0, 0, 0], textSize=1.2,
                                         replaceItemUniqueId=hud)
                hud2 = p.addUserDebugText(arts, [0, -1.2, 2.15], [0.2, 0.2, 0.6],
                                          textSize=1.1, replaceItemUniqueId=hud2)

            paso += 1
            espera = t0 + paso * DT - time.time()
            if espera > 0:
                time.sleep(espera)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
