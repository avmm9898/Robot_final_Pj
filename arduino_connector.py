import serial

# init serial
ser = serial.Serial("/dev/ttyACM0", 115200)


def arduino_write(s):
    # talk ot arduino and check
    ser.write(s.encode())
    while True:
        out = ser.readline().decode().strip()
        print(out)
        if "[echo]" not in out:
            continue
        if out == "[echo] " + s:
            break
        else:
            raise ValueError(out)
    return True


def arduino_write_easy(device, value=0):
    arduino_write(f"{device},{value};")


if __name__ == "__main__":
    # python3 arduino_connector.py a0 135
    import sys
    import time
    s = sys.argv[1:]
    arduino_write_easy(*s)
    # time.sleep(1)
