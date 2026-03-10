from machine import Pin, I2C
from pico_car import SSD1306_I2C
import time
i2c=I2C(1, scl=Pin(15),sda=Pin(14), freq=100000)
oled = SSD1306_I2C(128, 32, i2c)
oled.fill(0)
# Define a wider car pattern (each string is exactly 40 characters long)
car_pattern = [
    "            ****************            ",  # 40 characters
    "       ******             ******        ",  # 40 characters
    "       ******             ******        ",  # 40 characters
    "       ******             ******        ",  # 40 characters
    "     ******************************     ",  # 40 characters
    "    *                          *******  ",  # 40 characters
    "    *                          *******  ",  # 40 characters
    "    *                          *******  ",  # 40 characters
    "    *                          *******  ",  # 40 characters
    "    **********************************  ",  # 40 characters
    "        *****             *****         ",  # 40 characters
    "         ***               ***          "   # 40 characters
]


# Calculate starting positions to center the pattern on the display
pattern_width = len(car_pattern[0])   # Should be 40
pattern_height = len(car_pattern)       # Now 12 rows
start_x = (128 - pattern_width) // 2    # Center horizontally
start_y = (32 - pattern_height) // 2    # Center vertically

# Draw the car pattern using only the pixel() function
for row_index, row in enumerate(car_pattern):
    for col_index, char in enumerate(row):
        if char == "*":  # Turn on pixel wherever there is an asterisk
            oled.pixel(start_x + col_index, start_y + row_index, 1)

oled.show()  # Refresh the display to show the car

#
time.sleep(1)
oled.pixel(30, 30, 0)
oled.show()
