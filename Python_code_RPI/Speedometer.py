import socket
import struct
import pigpio
import time
# UDP config
UDP_IP = "0.0.0.0"  # all addresses 
UDP_PORT = 4444     # UDP Port
 
pi = pigpio.pi() #RPI gpio
if not pi.connected:
    print("cant connect with pigpio")
    exit()

#pwm pim
PWM_PIN = 18  # GPIO 18 (PWM)
pi.set_mode(PWM_PIN, pigpio.ALT5)

#socket setting
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sock.bind((UDP_IP, UDP_PORT))

print(f"listening {UDP_PORT}...")
#pi.set_PWM_dutycycle(PWM_PIN,125)
#pi.set_PWM_frequency(PWM_PIN, 0)


# UDP struct format 
STRUCT_FORMAT = "I 4s H c c f f f f f f f I I f f f 16s 16s i"

try:
    while True:
        data, addr = sock.recvfrom(1024)  # get udp UDP
        if len(data) != struct.calcsize(STRUCT_FORMAT):
            print("Wrong package...")
            continue

        # unpack
        unpacked_data = struct.unpack(STRUCT_FORMAT, data)

        speed = unpacked_data[5]  
        rpm = unpacked_data[6]
        engTemp = unpacked_data[8]
        fuel = unpacked_data[9]

        dash_lights = unpacked_data[12]
        show_lights = unpacked_data[13]
    
        DL_FULLBEAM = 2 ** 1  
        DL_HANDBRAKE = 2 ** 2 
        DL_TC = 2 ** 4
        DL_SIGNAL_L = 2 ** 5 
        DL_SIGNAL_R = 2 ** 6
        DL_OILWARN = 2 ** 8 
        DL_BATTERY = 2 ** 9 
        DL_ABS = 2 ** 10 

        full_beam_active = (show_lights & DL_FULLBEAM) != 0

        print(f"Speed received: {speed:.2f} m/s from")
        print(f"RPM received: {rpm:.2f} m/s from")
        print(f"Engine temp received: {engTemp:.2f} *C from")
        print(f"fuel: {fuel*100:.2f} % from")

        speed_in_km_h = speed*3.6
        print(f"speed in km/h: {speed_in_km_h:.2f} km/h")
        print(f"Full Beam {'ON' if (show_lights & DL_FULLBEAM) else 'OFF'}")
        print(f"Traction control {'ON' if (show_lights & DL_TC) else 'OFF'}")
        print(f"Left turn {'ON' if (show_lights & DL_SIGNAL_L) else 'OFF'}")
        print(f"Right turn {'ON' if (show_lights & DL_SIGNAL_R) else 'OFF'}")
        print(f"Warning light {'ON' if (show_lights & DL_SIGNAL_R & DL_SIGNAL_L) else 'OFF'}")
        print(f"Oil warning {'ON' if (show_lights & DL_OILWARN) else 'OFF'}")
        print(f"Battery warning {'ON' if (show_lights & DL_BATTERY) else 'OFF'}")
        print(f"ABS {'ON' if (show_lights & DL_ABS) else 'OFF'}")

        # PWM and speed borders 
        min_pwm = 100
        max_pwm = 1770
        step = 1
        max_speed = 255

        if speed > 0:
            pwm_frequency = int((speed_in_km_h / max_speed) * (max_pwm - min_pwm) + min_pwm)
        else:
            pwm_frequency = 100
        pi.hardware_PWM(PWM_PIN, pwm_frequency, 500000)  # duty cycle 50%
        #time.sleep(0.01)
        print(f"PWM: {pwm_frequency} Hz")

except KeyboardInterrupt:
    print("end...")
finally:
    pi.stop()
    sock.close()

