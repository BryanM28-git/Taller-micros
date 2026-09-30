# Taller Segundo Corte - Punto B (PC)
# Baxter (pybullet_robots) con cinematica inversa y agarre de objetos.
# Uso: python baxter_B_pots_esp32.py --puerto COM6 --repo pybullet_robots
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
PASO_IK = 4
V_OBJ_MAX = 0.30
W_ART_MAX = 1.5
DIST_AGARRE = 0.15
BASE_POS = [0.5, -0.8, 0.0]
BASE_YAW = math.pi / 2
LADOS = ("left", "right")
LADO = "left"
HOME = {"s0": 0.0, "s1": -0.55, "e0": 0.0, "e1": 0.75, "w0": 0.0, "w1": 1.26, "w2": 0.0}
MESA_Z = -0.30


RANGO_X = (-0.05, 0.75)
RANGO_Y = (-0.25, 0.45)
RANGO_Z = (-0.25, 0.35)


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
        self.pots, self.eventos, self.t = None, 0, 0.0
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
            if len(d) != 5 or d[0] != "P3":
                continue
            try:
                pots = np.array([float(v) for v in d[1:4]])
                ev = int(d[4])
            except ValueError:
                continue
            with self.lock:
                self.pots = np.clip(pots, 0.0, 1.0)
                self.eventos |= ev
                self.t = time.time()

    def leer(self):
        with self.lock:
            ev, self.eventos = self.eventos, 0
            vigente = self.pots is not None and time.time() - self.t < 1.5
            return (self.pots.copy() if vigente else None), ev

    def enviar(self, texto):
        try:
            self.ser.write(texto.encode())
        except (serial.SerialException, OSError):
            pass


def pots_a_mundo(pots):
    return np.array([r[0] + (r[1] - r[0]) * v
                     for r, v in zip((RANGO_X, RANGO_Y, RANGO_Z), pots)])


def buscar_urdf(repo):
    aqui = os.path.dirname(os.path.abspath(__file__))
    candidatos = [repo, os.path.join(aqui, repo), aqui, os.path.dirname(aqui),
                  pybullet_data.getDataPath()]
    nombres = ("toms_baxter.urdf", "baxter.urdf")
    revisadas = []
    for carpeta in candidatos:
        carpeta = os.path.abspath(carpeta)
        if carpeta in revisadas or not os.path.isdir(carpeta):
            continue
        revisadas.append(carpeta)
        for nombre in nombres:
            for raiz, _, archivos in os.walk(carpeta):
                if nombre in archivos:
                    ruta = os.path.join(raiz, nombre)
                    print("Baxter encontrado en:", ruta)
                    return ruta
    raise FileNotFoundError(
        "No encontre la URDF del Baxter. Carpetas revisadas:\n  " + "\n  ".join(revisadas) +
        "\nClona el repositorio dentro de la carpeta taller:\n"
        "  git clone https://github.com/erwincoumans/pybullet_robots.git")


def caja(medias, pos, color, masa=0.0, colision=True):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=medias) if colision else -1
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=medias, rgbaColor=color)
    return p.createMultiBody(masa, col, vis, pos)


def crear_escena():
    p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, -1],
               useFixedBase=True)
    alto = MESA_Z + 1.0
    caja([0.45, 0.28, alto / 2], [0.5, 0.05, -1 + alto / 2], [0.6, 0.45, 0.3, 1])
    caja([0.12, 0.12, 0.002], [0.5, 0.22, MESA_Z + 0.002], [0.1, 0.8, 0.2, 0.7],
         colision=False)
    objetos = []
    for pos, color in (([0.25, 0.0, MESA_Z + 0.03], [0.9, 0.1, 0.1, 1]),
                       ([0.75, 0.0, MESA_Z + 0.03], [0.1, 0.3, 0.9, 1])):
        oid = caja([0.025, 0.025, 0.025], pos, color, masa=0.1)
        p.changeDynamics(oid, -1, lateralFriction=1.0)
        objetos.append((oid, pos))
    return objetos


class Baxter:
    def __init__(self, urdf):
        self.id = p.loadURDF(urdf, BASE_POS, p.getQuaternionFromEuler([0, 0, BASE_YAW]),
                             useFixedBase=True)
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
                self.fmax[j] = info[10] if info[10] > 0 else 50.0

        self.brazo = {l: [self.jnt[f"{l}_{a}"] for a in HOME if f"{l}_{a}" in self.jnt]
                      for l in LADOS}
        self.ee = {"left": self._link(["left_gripper", "left_gripper_base", "left_hand"], 48),
                   "right": self._link(["right_gripper", "right_gripper_base", "right_hand"])}
        self.dedos = {l: [j for n, j in self.jnt.items()
                          if "finger" in n and (n.startswith(l[0] + "_gripper")
                                                or n.startswith(l + "_gripper"))
                          and j in self.movibles] for l in LADOS}
        print("Efector final:", {l: p.getJointInfo(self.id, e)[12].decode()
                                 for l, e in self.ee.items()})

        for l in LADOS:
            for a, v in HOME.items():
                if f"{l}_{a}" in self.jnt:
                    p.resetJointState(self.id, self.jnt[f"{l}_{a}"], v)
        self.q_obj = {j: p.getJointState(self.id, j)[0] for j in self.movibles}
        self.q_cmd = dict(self.q_obj)
        self.cid = None
        for l in LADOS:
            self.pinza(l, cerrar=False)

    def _link(self, candidatos, respaldo=None):
        for n in candidatos:
            if n in self.lnk:
                return self.lnk[n]
        if respaldo is not None:
            return respaldo
        raise KeyError(f"No encontre ninguno de estos links: {candidatos}")

    def pose_ee(self, lado):
        ls = p.getLinkState(self.id, self.ee[lado], computeForwardKinematics=True)
        return np.array(ls[4]), ls[5]

    def ik(self, lado, pos, orn=None):
        ll = [self.ll[j] for j in self.movibles]
        ul = [self.ul[j] for j in self.movibles]
        jr = [u - l for l, u in zip(ll, ul)]
        rp = [self.q_obj[j] for j in self.movibles]
        kw = dict(lowerLimits=ll, upperLimits=ul, jointRanges=jr, restPoses=rp,
                  maxNumIterations=100, residualThreshold=1e-4)
        if orn is None:
            sol = p.calculateInverseKinematics(self.id, self.ee[lado], list(pos), **kw)
        else:
            sol = p.calculateInverseKinematics(self.id, self.ee[lado], list(pos), orn, **kw)
        for k, j in enumerate(self.movibles):
            if j in self.brazo[lado]:
                self.q_obj[j] = float(np.clip(sol[k], self.ll[j], self.ul[j]))

    def pinza(self, lado, cerrar):
        for j in self.dedos[lado]:
            abierto = self.ul[j] if abs(self.ul[j]) > abs(self.ll[j]) else self.ll[j]
            self.q_obj[j] = float(np.clip(0.0, self.ll[j], self.ul[j])) if cerrar else abierto

    def agarrar(self, lado, objetos):
        ls = p.getLinkState(self.id, self.ee[lado], computeForwardKinematics=True)
        pe, oe = ls[0], ls[1]
        cercano, dmin = None, DIST_AGARRE
        for oid in objetos:
            d = np.linalg.norm(np.subtract(p.getBasePositionAndOrientation(oid)[0], pe))
            if d < dmin:
                cercano, dmin = oid, d
        if cercano is None:
            return False
        po, oo = p.getBasePositionAndOrientation(cercano)
        ip, io = p.invertTransform(pe, oe)
        rp, ro = p.multiplyTransforms(ip, io, po, oo)
        self.cid = p.createConstraint(self.id, self.ee[lado], cercano, -1, p.JOINT_FIXED,
                                      [0, 0, 0], rp, [0, 0, 0], ro, [0, 0, 0, 1])
        p.changeConstraint(self.cid, maxForce=200)
        return True

    def soltar(self):
        if self.cid is not None:
            p.removeConstraint(self.cid)
            self.cid = None

    def mover(self, dt):
        paso = W_ART_MAX * dt
        for j in self.movibles:
            self.q_cmd[j] += float(np.clip(self.q_obj[j] - self.q_cmd[j], -paso, paso))
        p.setJointMotorControlArray(self.id, self.movibles, p.POSITION_CONTROL,
                                    targetPositions=[self.q_cmd[j] for j in self.movibles],
                                    forces=[self.fmax[j] for j in self.movibles])


def main():
    ap = argparse.ArgumentParser(description="Punto B: Baxter con 3 potenciometros")
    ap.add_argument("--puerto", required=True)
    ap.add_argument("--repo", default="pybullet_robots", help="Carpeta de pybullet_robots")
    args = ap.parse_args()

    urdf = buscar_urdf(args.repo)
    esp = LectorESP32(args.puerto)

    p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(DT)
    p.setPhysicsEngineParameter(numSolverIterations=150)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    objetos = crear_escena()
    bx = Baxter(urdf)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.resetDebugVisualizerCamera(2.2, 0, -35, [0.4, 0.0, -0.2])

    for _ in range(100):
        bx.mover(DT)
        p.stepSimulation()


    esquinas = [np.array([x, y, z]) for x in RANGO_X for y in RANGO_Y for z in RANGO_Z]
    for a in esquinas:
        for b in esquinas:
            if np.count_nonzero(np.abs(a - b) > 1e-6) == 1:
                p.addUserDebugLine(a, b, [0.7, 0.7, 0.7], 1)

    objetivo = bx.pose_ee(LADO)[0]
    cerrada = False
    vs = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0.6, 0, 0.7])
    marcador = p.createMultiBody(0, -1, vs, objetivo)
    hud = p.addUserDebugText(" ", [-0.3, 0.6, 0.7], [0, 0, 0], textSize=1.2)

    paso, t0 = 0, time.time()
    try:
        while p.isConnected():
            pots, ev = esp.leer()

            if ev & 1:
                if cerrada:
                    bx.soltar()
                    bx.pinza(LADO, False)
                    cerrada = False
                    esp.enviar("G,0\n")
                else:
                    bx.pinza(LADO, True)
                    cerrada = True
                    if bx.agarrar(LADO, [o for o, _ in objetos]):
                        esp.enviar("G,1\n")
            if ev & 2:
                bx.soltar()
                bx.pinza(LADO, False)
                cerrada = False
                esp.enviar("G,0\n")
                for oid, pos in objetos:
                    p.resetBasePositionAndOrientation(oid, pos, [0, 0, 0, 1])


            if pots is not None:
                meta = pots_a_mundo(pots)
                dif = meta - objetivo
                n = np.linalg.norm(dif)
                paso_max = V_OBJ_MAX * DT
                objetivo = meta if n <= paso_max else objetivo + dif * paso_max / n

            if paso % PASO_IK == 0:
                bx.ik(LADO, objetivo)

            bx.mover(DT)
            p.stepSimulation()

            if paso % 8 == 0:
                p.resetBasePositionAndOrientation(marcador, objetivo, [0, 0, 0, 1])
            if paso % 60 == 0:
                texto = (f"Pinza: {'CERRADA' if cerrada else 'ABIERTA'} | "
                         f"Objeto: {'AGARRADO' if bx.cid is not None else '-'} | "
                         f"Objetivo=({objetivo[0]:.2f}, {objetivo[1]:.2f}, {objetivo[2]:.2f})"
                         + ("" if pots is not None else " | SIN SENAL ESP32"))
                hud = p.addUserDebugText(texto, [-0.3, 0.6, 0.7], [0, 0, 0], textSize=1.2,
                                         replaceItemUniqueId=hud)

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
