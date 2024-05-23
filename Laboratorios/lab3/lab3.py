import serial
import time

# Actualiza 'COM3' con el puerto correcto de tu Arduino
ser = serial.Serial('COM9', 9600)  # Asegúrate de usar el puerto correcto
time.sleep(2)

setpoint = 30.0
kp = 0.5
ki = 0.05
kd = 0.1

integral = 0
last_error = 0
last_time = time.time()

def pid_control(error, time_change):
    global integral, last_error
    integral += error * time_change
    integral = max(min(integral, 50), -50)  # Limita el término integral
    derivative = (error - last_error) / time_change
    output = kp * error + ki * integral + kd * derivative
    last_error = error
    return output

while True:
    try:
        # Leer la línea desde Arduino
        line = ser.readline().decode('utf-8').strip()
        if "Distance:" in line:
            distance = float(line.split(":")[1].strip())
            error = setpoint - distance
            current_time = time.time()
            time_change = current_time - last_time
            if time_change >= 0.05:  # Actualización a 50 ms
                output = pid_control(error, time_change)
                # Envía tanto el error como el valor del PID a Arduino
                ser.write(f"{error},{output}\n".encode('utf-8'))
                last_time = current_time
            print(f"Distance: {distance} cm, Setpoint: {setpoint} cm, Error: {error}, Output: {output}")

    except Exception as e:
        print(f"Error: {e}")
