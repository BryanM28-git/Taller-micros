# Taller Segundo Corte - Punto A (ESP32, MicroPython)
# Dron del punto A al B con un solo boton. La ESP32 genera la trayectoria.
# Boton: GPIO26 a GND | LED: GPIO2
# ESP32 -> PC: SP,x,y,z,estado | PC -> ESP32: P,x,y,z
import sys
import time
import math
import select
from machine import Pin


PIN_BOTON = 26
PIN_LED = 2
PERIODO_MS = 20
ALTURA = 1.0
V_MAX = 0.8
T_MIN = 2.0
TOL_LLEGADA = 0.12


PUNTO_A = (-1.0, -1.0)
PUNTO_B = (1.0, 1.0)

boton = Pin(PIN_BOTON, Pin.IN, Pin.PULL_UP)
led = Pin(PIN_LED, Pin.OUT)


sp = [PUNTO_A[0], PUNTO_A[1], 0.1]
p0 = list(sp)
p1 = list(sp)
t_ini = 0
T = 1.0
en_tray = False
estado = "ESPERA"
pos_dron = [0.0, 0.0, 0.0]
t_telem = None
rx = ""


lectura = 1
estable = 1
t_cambio = time.ticks_ms()


def boton_presionado():
    global lectura, estable, t_cambio
    v = boton.value()
    ahora = time.ticks_ms()
    if v != lectura:
        lectura = v
        t_cambio = ahora
    if time.ticks_diff(ahora, t_cambio) > 30 and v != estable:
        estable = v
        return v == 0
    return False


def distancia(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def min_jerk(tau):
    return tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau)


def iniciar_trayectoria(destino, nuevo_estado):
    global p0, p1, T, t_ini, en_tray, estado
    p0 = list(sp)
    p1 = list(destino)
    T = max(T_MIN, 1.875 * distancia(p0, p1) / V_MAX)
    t_ini = time.ticks_ms()
    en_tray = True
    estado = nuevo_estado


def actualizar_trayectoria():
    global en_tray, sp
    if not en_tray:
        return
    tau = time.ticks_diff(time.ticks_ms(), t_ini) / 1000.0 / T
    if tau >= 1.0:
        sp = list(p1)
        en_tray = False
        return
    s = min_jerk(tau)
    for k in range(3):
        sp[k] = p0[k] + (p1[k] - p0[k]) * s


def llego():
    if en_tray:
        return False
    if t_telem is None or time.ticks_diff(time.ticks_ms(), t_telem) > 500:
        return True
    return distancia(pos_dron, p1) < TOL_LLEGADA


poll = select.poll()
poll.register(sys.stdin, select.POLLIN)


def leer_serial():
    global rx, t_telem
    while poll.poll(0):
        c = sys.stdin.read(1)
        if c == "\n":
            if rx.startswith("P,"):
                d = rx[2:].split(",")
                if len(d) == 3:
                    try:
                        for k in range(3):
                            pos_dron[k] = float(d[k])
                        t_telem = time.ticks_ms()
                    except ValueError:
                        pass
            rx = ""
        elif c != "\r" and len(rx) < 60:
            rx += c


def actualizar_led():
    if estado == "ESPERA":
        led.value(0)
    elif en_tray or not llego():
        led.value((time.ticks_ms() // 150) % 2)
    else:
        led.value(1)


def main():
    global estado
    t_ultimo = time.ticks_ms()
    while True:
        leer_serial()


        if estado == "ESPERA" and t_telem is not None:
            iniciar_trayectoria((PUNTO_A[0], PUNTO_A[1], ALTURA), "DESPEGUE")


        if not en_tray:
            if estado in ("DESPEGUE", "B_A"):
                estado = "EN_A"
            elif estado == "A_B":
                estado = "EN_B"


        if boton_presionado():
            if estado == "EN_A" and llego():
                iniciar_trayectoria((PUNTO_B[0], PUNTO_B[1], ALTURA), "A_B")
            elif estado == "EN_B" and llego():
                iniciar_trayectoria((PUNTO_A[0], PUNTO_A[1], ALTURA), "B_A")


        ahora = time.ticks_ms()
        if time.ticks_diff(ahora, t_ultimo) >= PERIODO_MS:
            t_ultimo = ahora
            actualizar_trayectoria()
            sys.stdout.write("SP,%.3f,%.3f,%.3f,%s\n" % (sp[0], sp[1], sp[2], estado))
            actualizar_led()

        time.sleep_ms(1)


main()
