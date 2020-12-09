import numpy as np
import robot_rt
import serial
import serial.tools.list_ports
import struct
import time
import matplotlib.pyplot as plt

arduino_ports = [
    p.device
    for p in serial.tools.list_ports.comports()
    #if 'Arduino' in p.description
]
if not arduino_ports:
    raise IOError("No Arduino found")

def main():
    try:
        print("Trying on port ", end="")
        print(arduino_ports[0])
        ser = serial.Serial(arduino_ports[0], 115200)
    except:
        raise IOError("Could not open arduino!")

    robot = robot_rt.robot

    # Timeout
    timeout = 1000
    last_send = int(round(time.time() * 1000))

    # Start communication
    ser.write('OK'.encode())

    i = 0
    q = np.zeros(6)
    while True:
        try:
            if int(round(time.time() * 1000)) - last_send > timeout:
                last_send = int(round(time.time() * 1000))
                ser.write('OK'.encode())

            n_read = ser.inWaiting()
            if n_read == 6*4 : # 6 DOF and 4 bytes each
                for i in range(6):
                    data = ser.read(4)
                    q[i], = struct.unpack("<f", data) # Little-endian float-point

                print(q)

                # Convert to radians
                #q = (q*3.1415926535)/180

                robot.plot(q, optimize="O0")

                last_send = int(round(time.time() * 1000))
                ser.write('OK'.encode())
            elif n_read > 0:
                ser.reset_input_buffer()

            #print(n_read)
            time.sleep(0.1)

        except Exception as e:
            print("Angulos: ")
            print(angles)

            #f.close()

            print(e)
            print("Exiting simulation!")
            break    

if __name__ == "__main__":
    main()