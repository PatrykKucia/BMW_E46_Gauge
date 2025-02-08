import socket
import struct
import pigpio
import time
# Konfiguracja portu UDP
UDP_IP = "0.0.0.0"  # Odbiór na wszystkich interfejsach
UDP_PORT = 4444      # Port UDP

# Inicjalizacja pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Nie udało się połączyć z pigpio")
    exit()

# Ustawienie pinu PWM
PWM_PIN = 18  # GPIO 18 (PWM)
pi.set_mode(PWM_PIN, pigpio.ALT5)

# Inicjalizacja gniazda UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sock.bind((UDP_IP, UDP_PORT))

print(f"Nasłuchuję na porcie {UDP_PORT}...")
pi.set_PWM_dutycycle(PWM_PIN,125)
pi.set_PWM_frequency(PWM_PIN, 0)


# Format struktury UDP zgodny z #pragma pack(push, 1)
STRUCT_FORMAT = "I 4s H c c f f f f f f f I I f f f 16s 16s i"

try:
    while True:
        data, addr = sock.recvfrom(1024)  # Odbiór ramki UDP
        if len(data) != struct.calcsize(STRUCT_FORMAT):
            print("Błędna długość pakietu, pomijam...")
            continue

        # Rozpakowanie binarnej ramki do struktury
        unpacked_data = struct.unpack(STRUCT_FORMAT, data)

        speed = unpacked_data[5]  # Indeks 5 - wartość float prędkości
        print(f"Otrzymano prędkość: {speed:.2f} km/h od {addr}")
        speed_in_km_h = speed*3.6
        print(f"speed w km/h: {speed_in_km_h:.2f} km/h")
        # Przeliczenie prędkości na częstotliwość PWM
        min_pwm = 100
        max_pwm = 1770

# Ustalamy krok, w jakim będzie się zmieniać częstotliwość PWM (np. 10 Hz)
        step = 1

# Maksymalna prędkość (255 km/h)
        max_speed = 255

        if speed > 0:
            pwm_frequency = int((speed_in_km_h / max_speed) * (max_pwm - min_pwm) + min_pwm)

# Zaokrąglamy wynik do najbliższego kroku (inkrementacja o 'step')
           # pwm_frequency = round(pwm_frequency / step) * step

# Zapewnienie, że PWM jest w zakresie 1000 - 1770 Hz
            #pwm_frequency = max(min_pwm, min(max_pwm, pwm_frequency))
        else:
            pwm_frequency = 10  # Minimalna wartość
     # Smooth transition to new frequency    current_frequency = pi.get_PWM_frequency(PWM_PIN)
       # if current_frequency != pwm_frequency:
        #    step = 1  # Step size for frequency change
         #   if pwm_frequency > current_frequency:
          #      step = abs(step)
         #   else:
          #      step = -abs(step)

           # for freq in range(current_frequency, pwm_frequency, step):
            #    pi.set_PWM_frequency(PWM_PIN, freq)
             #   time.sleep(0.01)  # Small delay for smooth transition
       # pi.set_PWM_frequency(PWM_PIN, pwm_frequency)
        pi.hardware_PWM(PWM_PIN, pwm_frequency, 500000)  # Wypełnienie 50%
#        time.sleep(0.01)
        print(f"Ustawiono częstotliwość PWM: {pwm_frequency} Hz")

except KeyboardInterrupt:
    print("Zatrzymywanie...")
finally:
    pi.stop()
    sock.close()

