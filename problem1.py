# Setup
GPIO.setmode(GPIO.BCM)

# Ultrasonic sensor pins
TRIG_PIN = 23
ECHO_PIN = 24

# Motor PWM pin
MOTOR_PIN = 18

# Setup GPIO pins
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(MOTOR_PIN, GPIO.OUT)

# Set the PWM frequency for motor control
pwm = GPIO.PWM(MOTOR_PIN, 1000)  # 1 kHz frequency
pwm.start(0)  # Start with 0% duty cycle (motor off)

# Function to measure distance using the HC-SR04 sensor
def measure_distance():
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.1)

    # Trigger the sensor to send a pulse
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Wait for the echo to return
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        start_time = time.time()

    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        end_time = time.time()

    # Calculate distance
    pulse_duration = end_time - start_time
    distance = pulse_duration * 17150  # Convert time to distance in cm
    distance = round(distance, 2)  # Round to 2 decimal places
    return distance

# Function to control motor speed based on distance
def control_motor(distance):
    # Define speed control logic
    if distance < 10:
        pwm.ChangeDutyCycle(100)  # Full speed
    elif distance < 20:
        pwm.ChangeDutyCycle(75)   # 75% speed
    elif distance < 30:
        pwm.ChangeDutyCycle(50)   # 50% speed
    elif distance < 40:
        pwm.ChangeDutyCycle(25)   # 25% speed
    else:
        pwm.ChangeDutyCycle(0)    # Motor off if distance is greater than 40 cm

# Main loop
try:
    while True:
        distance = measure_distance()
        print(f"Distance: {distance} cm")

        control_motor(distance)

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Program stopped by user.")
    pwm.stop()
    GPIO.cleanup()
