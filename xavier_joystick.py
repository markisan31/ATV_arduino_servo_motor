import socket
import serial

class Xavier_server:
    def start():
        host = '10.10.0.232'
        port = 8081

        ser=serial.Serial(port = "/dev/ttyACM0", baudrate = 460800, rtscts = 0)

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((host,port))
        count = 0

        print("Server Started")
        while True:
            data, addr = s.recvfrom(256)

            print(f" {data[0]} {data[1]} {data[2]} {data[3]} {data[4]}")
            count += 1
            print(f"{count}")

            ser.write(data)

            data = 0

        s.close()
        ser.close()