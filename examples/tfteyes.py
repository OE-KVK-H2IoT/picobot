import gc9a01
from machine import Pin, SPI

# Initialize SPI bus
spi = SPI(0, baudrate=60_000_000, sck=Pin(18), mosi=Pin(19))

# Initialize display
tft = gc9a01.GC9A01(
    spi,
    dc=Pin(16, Pin.OUT),
    cs=Pin(17, Pin.OUT),
    reset=Pin(20, Pin.OUT),
    backlight=Pin(21, Pin.OUT),
    width=240,
    height=240,
    rotation=0
)

# Initialize and clear screen
tft.init()
tft.fill(gc9a01.BLACK)
