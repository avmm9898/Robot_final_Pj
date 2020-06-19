import serial

# init serial
ser = serial.Serial("/dev/ttyACM0", 115200)


def writeArduino(s):
    """
    Talk to arduino and wait the echo.

    Parameters
    ----------
    s: str
        The brief string send to arduino

    Raises
    ------
    ValueError
        If the echo did not match the input, which
        indicate the connectaion may fail.
    """
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


def writeArduinoEasy(device, value=0):
    """ A simple wrap function """
    writeArduino(f"{device},{value};")


def setArduinoArm(device, value=0):
    """
    Set Arduino Arm movement

    Parameters
    ----------
    device: Integer
        The index of arm you want to move
    value: Integer
        The angle of arm you want to set

    Raises
    ------
    ValueError
        See writeArduino
    """
    writeArduino(f"a{device},{value};")


def setArduinoCar(mode, speed, distance=0):
    """
    Set Arduino Arm movement

    Parameters
    ----------
    mode: Char
        A single character to indicate mode
        `F` for forward,
        `B` for backward,
        `R` for right,
        `L` for left,
    speed: Integer
        The rotation speed(mm/s)

    distance: Integer
        The distance you want to go(mm)

    Raises
    ------
    ValueError
        See writeArduino
    """
    writeArduino(f"c{mode[0]}{speed},{distance};")


if __name__ == "__main__":
    # python3 arduino_connector.py a0 135
    import sys
    s = sys.argv[1:]
    writeArduinoEasy(*s)
