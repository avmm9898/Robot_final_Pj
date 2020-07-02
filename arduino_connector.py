import serial

# init serial
ser = serial.Serial("COM45", baudrate = 57600)
#ser = serial.Serial("/dev/ttyACM0", 57600)
ser.close()
speedtext = ""


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
    ser.open()
    # print("Send: ", s)
    ser.write(s.encode())
    while True:
        # out = ser.readline().decode().strip()
        out = ser.read_until(';'.encode()).decode().strip()
        # print(out)
        if "[speed]" in out:
            global speedtext
            speedtext = out
        if "[echo]" not in out:
            continue
        if out == "[echo] " + s:
            break
        else:
            ser.close()
            raise ValueError(out)
    ser.close()
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


def setArduinoSpeed(direction, speed, wheel):
    """
    Set Arduino Wheel speed

    Parameters
    ----------
    Direction: Char
        A single character to indicate direction
        `F` for forward,
        `B` for backward,
        `R` for right,
        `L` for left,
    speed: Integer
        The rotation speed(mm/s)
    wheel: char
        A or B wheel you want to set

    Raises
    ------
    ValueError
        See writeArduino
    """
    writeArduino(f"w{direction[0]}{wheel[0]},{speed};")


def getArduinoSpeed():
    """
    Get Arduino Wheel speed

    Returns 
    ------
    Speed: (Integer, Interger)
        The speed in mm of A and B motor

    Raises
    ------
    ValueError
        See writeArduino
    """
    writeArduino("s;")
    global speedtext
    print(speedtext)
    if speedtext:
        speedA, speedB = speedtext.split(']')[1].split(',')
        return int(speedA), int(speedB)
    speedtext = ""


if __name__ == "__main__":
    # python3 arduino_connector.py a0 135
    # setArduinoArm(2, 0)
    # setArduinoCar('R', 200, 100)
    # setArduinoSpeed('F', 100, 'A')
    # print(getArduinoSpeed())
    import sys
    s = sys.argv[1:]
    writeArduinoEasy(*s)
