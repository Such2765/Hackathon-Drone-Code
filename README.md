# Hackathon-Drone-Code

# SETTINGS
# Motor Pins (BCM)
pin_motor_1 = 2     # FL, CW
pin_motor_2 = 28    # FR, CCW
pin_motor_3 = 15    # RL, CCW
pin_motor_4 = 16    # RR, CW

# Servo Pins
pin_servo_1 = 6     # FL arm
pin_servo_2 = 7     # FR arm
pin_servo_3 = 8     # RL arm
pin_servo_4 = 9     # RR arm

# Signal Extender Pin
pin_extender_signal = 10

# IMU I2C Pins
pin_i2c_sda = 12
pin_i2c_scl = 13

# RC Receiver UART (0 or 1)
uart_channel_rc = 1

# Physical & Flight Parameters
servo_pos_folded_ns:int = 700000
servo_pos_open_ns:int = 2300000

# Throttle settings
min_throttle:float = 0.14
max_throttle_limit:float = 0.22

# Max Angular Rates (dps)
max_roll_dps:float = 30.0
max_pitch_dps:float = 30.0
max_yaw_dps:float = 50.0

# Loop Frequency (Hz)
loop_frequency_hz:float = 250.0

# PID Gains
roll_kp:float = 0.00043714285
roll_ki:float = 0.00255
roll_kd:float = 0.00002571429
pitch_kp:float = roll_kp
pitch_ki:float = roll_ki
pitch_kd:float = roll_kd
yaw_kp:float = 0.001714287
yaw_ki:float = 0.003428571
yaw_kd:float = 0.0

# MAIN FUNCTION 

import machine
import time
import ibus
import toolkit

def run() -> None:
    """ Main flight controller function """

    print("Hello from Scout Flight Controller!")

    # Signal startup with LED
    status_led = machine.Pin(25, machine.Pin.OUT)
    for _ in range(8):
        status_led.on()
        time.sleep(0.1)
        status_led.off()
        time.sleep(0.1)

    # Wait for IMU to settle
    print("Waiting 3 seconds for the IMU to settle...")
    time.sleep(3)

    # Overclock for a performance
    machine.freq(250000000)
    print("Overclocked to 250MHz")

    # Set up RC receiver
    receiver:ibus.IBus = ibus.IBus(uart_channel_rc)
    print("RC receiver set up")

    # Initial Safety Validations
    print("Performing startup safety checks...")
    rx_input = receiver.read()
    if rx_input[5] == 2000: # Flight mode
        FATAL_ERROR("Flight mode is ON at startup. Please disarm.")
    if rx_input[6] != 2000: # Arm position
        FATAL_ERROR("Arms are not DEPLOYED at startup. Please deploy.")
    print("Safety checks passed.")

    print(f"Roll PID: Kp={roll_kp}, Ki={roll_ki}, Kd={roll_kd}")
    print(f"Pitch PID: Kp={pitch_kp}, Ki={pitch_ki}, Kd={pitch_kd}")
    print(f"Yaw PID: Kp={yaw_kp}, Ki={yaw_ki}, Kd={yaw_kd}")

    # Set up IMU (MPU-6050)
    i2c_bus = machine.I2C(0, sda=machine.Pin(pin_i2c_sda), scl=machine.Pin(pin_i2c_scl))
    imu_address:int = 0x68
    i2c_bus.writeto_mem(imu_address, 0x6B, bytes([0x01])) # Wake up
    i2c_bus.writeto_mem(imu_address, 0x1A, bytes([0x05])) # Set LPF
    i2c_bus.writeto_mem(imu_address, 0x1B, bytes([0x08])) # Set gyro scale

    # Validate IMU connection
    device_id = i2c_bus.readfrom_mem(imu_address, 0x75, 1)[0]
    if device_id == 104:
        print("MPU-6050 WHOAMI validated!")
    else:
        FATAL_ERROR(f"MPU-6050 WHOAMI failed! Got '{device_id}' instead of 104.")

    # Measure gyro bias
    print("Measuring gyro bias...")
    gyro_x_samples, gyro_y_samples, gyro_z_samples = [], [], []
    for _ in range(120):
        raw_gyro_bytes = i2c_bus.readfrom_mem(imu_address, 0x43, 6)
        gyro_x_samples.append(translate_pair(raw_gyro_bytes[0], raw_gyro_bytes[1]) / 65.5)
        gyro_y_samples.append(translate_pair(raw_gyro_bytes[2], raw_gyro_bytes[3]) / 65.5)
        gyro_z_samples.append((translate_pair(raw_gyro_bytes[4], raw_gyro_bytes[5]) / 65.5) * -1)
        time.sleep(0.025)
    gyro_offset_x = sum(gyro_x_samples) / len(gyro_x_samples)
    gyro_offset_y = sum(gyro_y_samples) / len(gyro_y_samples)
    gyro_offset_z = sum(gyro_z_samples) / len(gyro_z_samples)
    print(f"Gyro bias: X={gyro_offset_x:.2f}, Y={gyro_offset_y:.2f}, Z={gyro_offset_z:.2f}")

    # Set up Motor PWMs
    M1, M2, M3, M4 = [machine.PWM(machine.Pin(p)) for p in [pin_motor_1, pin_motor_2, pin_motor_3, pin_motor_4]]
    for motor in [M1, M2, M3, M4]:
        motor.freq(250)
    print("Motor PWMs set up @ 250 Hz")
    
    # Set up Servo PWMs
    S1, S2, S3, S4 = [machine.PWM(machine.Pin(p)) for p in [pin_servo_1, pin_servo_2, pin_servo_3, pin_servo_4]]
    for servo in [S1, S2, S3, S4]:
        servo.freq(50)
        servo.duty_ns(servo_pos_open_ns)
    print("Servo PWMs set up @ 50 Hz")

    # Set up Signal Extender GPIO
    extender_gpio = machine.Pin(pin_extender_signal, machine.Pin.OUT)
    extender_gpio.low()
    print("Signal extender pin initialized.")

    # Pre-Loop Calculations
    loop_period_sec:float = 1.0 / loop_frequency_hz
    loop_period_us:int = int(round(loop_period_sec * 1000000, 0))
    usable_throttle_range:float = (max_throttle_limit or 1.0) - min_throttle
    integral_limit:float = 150.0
    is_armed_prev_loop:bool = False

    # PID state variables
    integral_roll, prev_error_roll = 0.0, 0.0
    integral_pitch, prev_error_pitch = 0.0, 0.0
    integral_yaw, prev_error_yaw = 0.0, 0.0

    # Infinite Flight Control Loop
    status_led.on()
    print("\n-- BEGINNING FLIGHT CONTROL LOOP --\n")
    try:
        while True:
            loop_start_time_us = time.ticks_us()

            # Read and correct IMU data
            raw_gyro_bytes = i2c_bus.readfrom_mem(imu_address, 0x43, 6)
            rate_roll = ((translate_pair(raw_gyro_bytes[0], raw_gyro_bytes[1]) / 65.5) - gyro_offset_x) * -1
            rate_pitch = (translate_pair(raw_gyro_bytes[2], raw_gyro_bytes[3]) / 65.5) - gyro_offset_y
            rate_yaw = ((translate_pair(raw_gyro_bytes[4], raw_gyro_bytes[5]) / 65.5) * -1) - gyro_offset_z

            # Read and normalize RC inputs
            rx_input = receiver.read()
            stick_throttle = normalize(rx_input[3], 1000.0, 2000.0, 0.0, 1.0)
            stick_pitch = normalize(rx_input[2], 1000.0, 2000.0, 1.0, -1.0) # Inverted
            stick_roll = normalize(rx_input[1], 1000.0, 2000.0, -1.0, 1.0)
            stick_yaw = normalize(rx_input[4], 1000.0, 2000.0, -1.0, 1.0)
            
            # Ancillary Systems
            extender_gpio.value(1 if rx_input[7] == 2000 else 0)

            # Primary Systems (Flight Mode Dependent)
            if rx_input[5] == 1000: # Standby (Disarmed)
                duty_0_percent = calculate_duty_cycle(0.0)
                for motor in [M1, M2, M3, M4]: motor.duty_ns(duty_0_percent)

                integral_roll, integral_pitch, integral_yaw = 0.0, 0.0, 0.0
                
                servo_target_pos_ns = servo_pos_folded_ns if rx_input[6] == 1000 else servo_pos_open_ns
                for servo in [S1, S2, S3, S4]: servo.duty_ns(servo_target_pos_ns)

                is_armed_prev_loop = False

            elif rx_input[5] == 2000: # Flight (Armed)
                if rx_input[6] != 2000: # Safety: arms must be deployed
                    continue

                if not is_armed_prev_loop and stick_throttle > 0.05: # Safety: throttle zero on arm
                    FATAL_ERROR("Throttle must be zero when arming.")
                
                # PID Controller
                current_throttle = min_throttle + (usable_throttle_range * stick_throttle)

                error_roll = (stick_roll * max_roll_dps) - rate_roll
                error_pitch = (stick_pitch * max_pitch_dps) - rate_pitch
                error_yaw = (stick_yaw * max_yaw_dps) - rate_yaw

                # Roll PID
                p_term_roll = error_roll * roll_kp
                integral_roll += error_roll * roll_ki * loop_period_sec
                integral_roll = max(min(integral_roll, integral_limit), -integral_limit)
                d_term_roll = roll_kd * (error_roll - prev_error_roll) / loop_period_sec
                output_roll = p_term_roll + integral_roll + d_term_roll

                # Pitch PID
                p_term_pitch = error_pitch * pitch_kp
                integral_pitch += error_pitch * pitch_ki * loop_period_sec
                integral_pitch = max(min(integral_pitch, integral_limit), -integral_limit)
                d_term_pitch = pitch_kd * (error_pitch - prev_error_pitch) / loop_period_sec
                output_pitch = p_term_pitch + integral_pitch + d_term_pitch
                
                # Yaw PID
                p_term_yaw = error_yaw * yaw_kp
                integral_yaw += error_yaw * yaw_ki * loop_period_sec
                integral_yaw = max(min(integral_yaw, integral_limit), -integral_limit)
                d_term_yaw = yaw_kd * (error_yaw - prev_error_yaw) / loop_period_sec
                output_yaw = p_term_yaw + integral_yaw + d_term_yaw

                # Motor Mixing
                m_thrust_1 = current_throttle + output_pitch + output_roll - output_yaw
                m_thrust_2 = current_throttle + output_pitch - output_roll + output_yaw
                m_thrust_3 = current_throttle - output_pitch + output_roll + output_yaw
                m_thrust_4 = current_throttle - output_pitch - output_roll - output_yaw
                
                M1.duty_ns(calculate_duty_cycle(m_thrust_1))
                M2.duty_ns(calculate_duty_cycle(m_thrust_2))
                M3.duty_ns(calculate_duty_cycle(m_thrust_3))
                M4.duty_ns(calculate_duty_cycle(m_thrust_4))

                prev_error_roll, prev_error_pitch, prev_error_yaw = error_roll, error_pitch, error_yaw
                is_armed_prev_loop = True
            
            # Maintain loop frequency
            loop_duration_us = time.ticks_diff(time.ticks_us(), loop_start_time_us)
            if loop_duration_us < loop_period_us:
                time.sleep_us(loop_period_us - loop_duration_us)

    except Exception as e:
        # On failure, cut motor power
        for motor in [M1, M2, M3, M4]: motor.deinit()
        FATAL_ERROR(str(e))

# Utility Functions
def calculate_duty_cycle(throttle:float, dead_zone:float = 0.03) -> int:
    """ Converts throttle (0.0-1.0) to PWM duty cycle in nanoseconds."""
    duty_ceiling, duty_floor = 2000000, 1000000
    if throttle < dead_zone: return duty_floor
    percentage = (throttle - dead_zone) / (1.0 - dead_zone)
    dutyns = duty_floor + ((duty_ceiling - duty_floor) * percentage)
    return int(max(duty_floor, min(dutyns, duty_ceiling)))

def normalize(raw_value, in_min, in_max, out_min, out_max) -> float:
    """ Scales a value from one range to another."""
    return out_min + ((out_max - out_min) * ((raw_value - in_min) / (in_max - in_min)))

def translate_pair(high_byte:int, low_byte:int) -> int:
    """ Converts a high/low byte pair into a single signed integer."""
    value = (high_byte << 8) | low_byte
    return value - 65536 if value >= 0x8000 else value

def FATAL_ERROR(error_message:str):
    """ Handles a critical error by halting and blinking the LED."""
    print("\n" + "#"*20)
    print(f"FATAL ERROR: {error_message}")
    print("#"*20 + "\n")
    led = machine.Pin(25, machine.Pin.OUT)
    while True:
        led.on()
        time.sleep(0.5)
        led.off()
        time.sleep(0.5)

# Start the Program
run()
