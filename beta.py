import serial
import time
import sys
import glob


def serial_ports():
    """ Lists serial port names
        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')
    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result
  
# Определяем функцию для отправки команды MSP
def send_msp(cmd):
    header = b'$M<'
    size = len(cmd)

    checksum = 0
    for c in cmd:
        checksum ^= c

    cmd_with_checksum = header + bytes([size]) + cmd + bytes([checksum])
    #print (cmd_with_checksum)
    ser.write(cmd_with_checksum)  
    
if __name__ == '__main__':
    print(serial_ports())
    # Открываем последовательный порт
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=5) #'/dev/ttyACM0'

    # Отправляем команду "version"
    ser.write(b'$M<version\r\n')

    # Ждем ответа
    time.sleep(5)

    # Читаем ответ
    response = ser.read(ser.inWaiting())

    # Закрываем порт
    ser.close()

    # Выводим ответ
    print(response)

