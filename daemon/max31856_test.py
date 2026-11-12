import time
import board
import busio
import digitalio
import adafruit_max31856

spi = None
cs = None

try:
    # Initialize SPI
    spi = busio.SPI(board.SCK, board.MOSI, board.MISO)

    # GPIO 0 is the MAX31856 chip-select pin.
    # GPIO 5 is reserved for the kiln SSR!
    cs = digitalio.DigitalInOut(board.D0)
    cs.direction = digitalio.Direction.OUTPUT
    cs.value = True  # CS inactive

    thermocouple = adafruit_max31856.MAX31856(spi, cs)

    print("MAX31856 fault monitoring started. Press Ctrl+C to exit.")

    while True:
        print(thermocouple.fault)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nCtrl+C detected. Stopping.")

except Exception as e:
    print(f"\nError: {e}")
    raise

finally:
    print("Cleaning up GPIO/SPI resources...")

    if cs is not None:
        try:
            cs.value = True  # Deselect MAX31856
        finally:
            cs.deinit()

    if spi is not None:
        spi.deinit()

    print("Cleanup complete.")
