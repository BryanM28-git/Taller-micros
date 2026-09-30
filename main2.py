# Taller Segundo Corte - Punto B (ESP32, MicroPython)
# Consola para Baxter: 3 potenciometros (X, Y, Z de la pinza) + 1 boton.
# POT X: GPIO34 | POT Y: GPIO35 | POT Z: GPIO32 | Boton: GPIO26 a GND | LED: GPIO2
# Boton corto: pinza | Boton largo (>1 s): reiniciar objetos
# ESP32 -> PC: P3,x,y,z,ev | PC -> ESP32: G,1 / G,0
import sys
import time
import select
from machine import Pin, ADC

PERIODO_MS = 20
ALFA = 0.15
HISTERESIS = 0.004
T_LARGO_MS = 1000


class Potenciometro:
    def __init__(self, pin):
        self.adc = ADC(Pin(pin))
        self.adc.atten(ADC.ATTN_11DB)
        self.filt = self.crudo()
        self.salida = self.filt

    def crudo(self):
        return (self.adc.read_u16() >> 4) / 4095.0

    def leer(self):
        m = sum(self.crudo() for _ in range(4)) / 4.0
        self.filt += ALFA * (m - self.filt)
        if abs(self.filt - self.salida) > HISTERESIS:
            self.salida = self.filt
        return self.salida


pots = [Potenciometro(34), Potenciometro(35), Potenciometro(32)]
boton = Pin(26, Pin.IN, Pin.PULL_UP)
led = Pin(2, Pin.OUT)


lectura = 1
estable = 1
t_cambio = time.ticks_ms()
t_presion = 0
largo_enviado = False
eventos = 0

objeto_agarrado = False
rx = ""
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)


def revisar_boton():
    global lectura, estable, t_cambio, t_presion, largo_enviado, eventos
    v = boton.value()
    ahora = time.ticks_ms()
    if v != lectura:
        lectura = v
        t_cambio = ahora
    if time.ticks_diff(ahora, t_cambio) > 25 and v != estable:
        estable = v
        if v == 0:
            t_presion = ahora
            largo_enviado = False
        elif not largo_enviado:
            eventos |= 1
    if estable == 0 and not largo_enviado and\
            time.ticks_diff(ahora, t_presion) > T_LARGO_MS:
        eventos |= 2
        largo_enviado = True


def leer_serial():
    global rx, objeto_agarrado
    while poll.poll(0):
        c = sys.stdin.read(1)
        if c == "\n":
            if rx.startswith("G,"):
                objeto_agarrado = rx[2:3] == "1"
            rx = ""
        elif c != "\r" and len(rx) < 20:
            rx += c


def main():
    global eventos
    led.value(1)
    time.sleep_ms(300)
    led.value(0)
    t_ultimo = time.ticks_ms()
    while True:
        revisar_boton()
        leer_serial()

        ahora = time.ticks_ms()
        if time.ticks_diff(ahora, t_ultimo) >= PERIODO_MS:
            t_ultimo = ahora
            x, y, z = [p.leer() for p in pots]
            sys.stdout.write("P3,%.3f,%.3f,%.3f,%d\n" % (x, y, z, eventos))
            eventos = 0
            if objeto_agarrado:
                led.value(1)
            else:
                led.value(1 if (ahora % 1000) < 80 else 0)

        time.sleep_ms(1)


main()
