# Taller Segundo Corte - Punto A (PC)
# Dron CF2X en gym-pybullet-drones que sigue el setpoint enviado por la ESP32.
# Uso: python dron_A_esp32.py --puerto COM6
import argparse
import threading
import time

import numpy as np
import pybullet as p
import serial

from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync


PUNTO_A = (-1.0, -1.0)
PUNTO_B = (1.0, 1.0)
POS_INICIAL = np.array([[PUNTO_A[0], PUNTO_A[1], 0.1]])
TIMEOUT = 1.5
TEXTOS = {"ESPERA": "Esperando ESP32", "DESPEGUE": "Despegando en A",
          "EN_A": "En el punto A (presiona el boton)", "A_B": "Volando de A a B",
          "EN_B": "En el punto B (presiona el boton)", "B_A": "Volando de B a A"}


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
        self.sp, self.estado, self.t = None, "ESPERA", 0.0
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
            if len(d) != 5 or d[0] != "SP":
                continue
            try:
                sp = np.array([float(v) for v in d[1:4]])
            except ValueError:
                continue
            with self.lock:
                self.sp, self.estado, self.t = sp, d[4], time.time()

    def leer(self):
        with self.lock:
            vigente = self.sp is not None and time.time() - self.t < TIMEOUT
            return (self.sp.copy() if vigente else None), self.estado

    def enviar(self, texto):
        try:
            self.ser.write(texto.encode())
        except (serial.SerialException, OSError):
            pass


def dibujar_escena(cli):
    for nombre, (x, y), color in (("A", PUNTO_A, [1, 0.2, 0.2]), ("B", PUNTO_B, [0.2, 0.8, 0.2])):
        vs = p.createVisualShape(p.GEOM_CYLINDER, radius=0.2, length=0.02,
                                 rgbaColor=color + [0.8], physicsClientId=cli)
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs, basePosition=[x, y, 0.01],
                          physicsClientId=cli)
        p.addUserDebugLine([x, y, 0], [x, y, 1.6], color, 1.5, physicsClientId=cli)
        p.addUserDebugText(f"Punto {nombre}", [x, y, 1.7], color, textSize=1.6,
                           physicsClientId=cli)
    p.addUserDebugLine([PUNTO_A[0], PUNTO_A[1], 1.0], [PUNTO_B[0], PUNTO_B[1], 1.0],
                       [0.6, 0.6, 0.6], 1, physicsClientId=cli)
    vs = p.createVisualShape(p.GEOM_SPHERE, radius=0.04, rgbaColor=[1, 0.6, 0, 0.9],
                             physicsClientId=cli)
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs,
                             basePosition=POS_INICIAL[0], physicsClientId=cli)


def main():
    ap = argparse.ArgumentParser(description="Punto A: dron de A a B con ESP32")
    ap.add_argument("--puerto", required=True, help="Puerto de la ESP32, ej: COM6")
    args = ap.parse_args()

    esp = LectorESP32(args.puerto)

    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=1,
                     initial_xyzs=POS_INICIAL, initial_rpys=np.zeros((1, 3)),
                     physics=Physics.PYB, pyb_freq=240, ctrl_freq=48,
                     gui=True, record=False, obstacles=False, user_debug_gui=False)
    cli = env.getPyBulletClient()
    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

    p.resetDebugVisualizerCamera(4.0, 45, -30, [0, 0, 0.6], physicsClientId=cli)
    marcador = dibujar_escena(cli)
    hud = p.addUserDebugText(" ", [-1.5, 1.5, 2.0], [0, 0, 0], textSize=1.4,
                             physicsClientId=cli)

    objetivo = POS_INICIAL[0].copy()
    accion = np.zeros((1, 4))
    pos_prev = POS_INICIAL[0].copy()
    i, inicio = 0, time.time()
    try:
        while p.isConnected(cli):
            obs = env.step(accion)[0]
            estado = obs[0]
            pos = estado[0:3]

            sp, etapa = esp.leer()
            if sp is not None:
                objetivo = sp

            accion[0, :], _, _ = ctrl.computeControlFromState(
                control_timestep=env.CTRL_TIMESTEP, state=estado,
                target_pos=objetivo, target_rpy=np.zeros(3))

            if i % 4 == 0:
                p.resetBasePositionAndOrientation(marcador, objetivo, [0, 0, 0, 1],
                                                  physicsClientId=cli)
                p.addUserDebugLine(pos_prev, pos, [0.1, 0.4, 1], 2, lifeTime=15,
                                   physicsClientId=cli)
                pos_prev = pos.copy()
            if i % 5 == 0:
                esp.enviar(f"P,{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}\n")
            if i % 24 == 0:
                texto = (TEXTOS.get(etapa, etapa) +
                         f" | pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})" +
                         ("" if sp is not None else " | SIN SENAL ESP32"))
                hud = p.addUserDebugText(texto, [-1.5, 1.5, 2.0], [0, 0, 0], textSize=1.4,
                                         replaceItemUniqueId=hud, physicsClientId=cli)

            i += 1
            sync(i, inicio, env.CTRL_TIMESTEP)
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
