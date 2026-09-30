# Taller Segundo Corte - Punto C (ESP32, MicroPython)
# Consola articular para Atlas: 3 potenciometros + 1 boton, 4 grupos de articulaciones.
# POT1: GPIO34 | POT2: GPIO35 | POT3: GPIO32 | Boton: GPIO26 a GND | LED: GPIO2
# Boton corto: siguiente grupo | Boton largo (>1 s): cambiar lado
# ESP32 -> PC: C3,lado,grupo,p1,p2,p3
import sys
import time
from machine import Pin, ADC

PERIODO_MS = 20
ALFA = 0.15
HISTERESIS = 0.004
T_LARGO_MS = 1000
NUM_GRUPOS = 4


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


grupo = 0
brazo = 0


lectura = 1
estable = 1
t_cambio = time.ticks_ms()
t_presion = 0
largo_hecho = False


def revisar_boton():
    global lectura, estable, t_cambio, t_presion, largo_hecho, grupo, brazo
    v = boton.value()
    ahora = time.ticks_ms()
    if v != lectura:
        lectura = v
        t_cambio = ahora
    if time.ticks_diff(ahora, t_cambio) > 25 and v != estable:
        estable = v
        if v == 0:
            t_presion = ahora
            largo_hecho = False
        elif not largo_hecho:
            grupo = (grupo + 1) % NUM_GRUPOS
    if estable == 0 and not largo_hecho and\
            time.ticks_diff(ahora, t_presion) > T_LARGO_MS:
        brazo = 1 - brazo
        largo_hecho = True


def actualizar_led():
    t = time.ticks_ms() % 2000
    periodo = 400
    encendido = 120 if brazo == 0 else 280
    n = grupo + 1
    led.value(1 if (t < n * periodo and (t % periodo) < encendido) else 0)


def main():
    led.value(1)
    time.sleep_ms(300)
    led.value(0)
    t_ultimo = time.ticks_ms()
    while True:
        revisar_boton()
        ahora = time.ticks_ms()
        if time.ticks_diff(ahora, t_ultimo) >= PERIODO_MS:
            t_ultimo = ahora
            p1, p2, p3 = [p.leer() for p in pots]
            sys.stdout.write("C3,%d,%d,%.3f,%.3f,%.3f\n" % (brazo, grupo, p1, p2, p3))
            actualizar_led()
        time.sleep_ms(1)


main()
