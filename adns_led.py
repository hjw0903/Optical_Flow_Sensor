#!/usr/bin/env python3
# NeoPixel library strandtest example, Tony DiCola (tony@tonydicola.com)
# Modified by Jungwon Hwang (hjw0903@gmail.com) for using the ADNS sensor light

import time
from neopixel import *

'''
LED strip configuration:

Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
LED_COUNT : Number of LED pixels.
LED_PIN   : 18 uses PWM!
LED_FREQ_HZ : LED signal frequency in hertz (usually 800khz)
LED_DMA   : DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS : Set to 0 for darkest and 255 for brightest
LED_INVERT : True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL : set to '1' for GPIOs 13, 19, 41, 45 or 53
'''

# Define functions which animate LEDs in various ways.
def color_wipe(strip, color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms/1000.0)
