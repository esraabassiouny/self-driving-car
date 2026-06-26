import serial
import time
import config

class SerialController:
    def __init__(self):
        self.port = config.SERIAL_PORT
        self.baud = config.SERIAL_BAUD
        self.ser = None
        self.last_reconnect_time = 0.0
        self.last_send_time = 0.0
        self.initialize_serial()

    def initialize_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            time.sleep(3)  # Wait for Arduino to initialize
            print("✅ Serial initialized successfully!")
        except Exception as e:
            print(f"❌ Initial serial connection failed: {e}")
            self.ser = None

    def read_distance(self, default_dist=999.0):
        """Reads non-blocking telemetry from Arduino and returns the latest distance."""
        if self.ser is None or not self.ser.is_open:
            return default_dist

        latest_dist = default_dist
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if "DIST:" in line:
                    try:
                        latest_dist = float(line.split(":")[1].strip())
                    except ValueError:
                        pass
        except Exception as e:
            print(f"⚠️ Error reading serial telemetry: {e}")
        return latest_dist

    def safe_write(self, data):
        """Safely writes data, automatically reconnecting if connection fails."""
        if self.ser is None or not self.ser.is_open:
            current_time = time.time()
            if current_time - self.last_reconnect_time > 3.0:
                self.last_reconnect_time = current_time
                try:
                    if self.ser is not None:
                        self.ser.close()
                except:
                    pass

                try:
                    self.ser = serial.Serial(
                        self.port,
                        self.baud,
                        timeout=0,
                        write_timeout=0.1
                    )
                    time.sleep(0.5)
                    print("✅ Serial reconnected successfully!")
                except Exception as reconnect_error:
                    print(f"❌ Reconnection failed: {reconnect_error}")
            return False

        try:
            self.ser.write(data)
            return True
        except (serial.SerialException, OSError) as e:
            print(f"⚠️ Serial write failed: {e}")
            try:
                self.ser.close()
            except:
                pass
            return False

    def send_motor_speeds(self, left_pwm, right_pwm, current_state, min_dist, force=False):
        """Sends rate-limited motor speeds (LxxxRxxx) to Arduino."""
        current_time = time.time()
        # Rate limit to 20Hz (every 50ms) unless forced
        if force or (current_time - self.last_send_time >= 0.05):
            left_pwm = int(left_pwm)
            right_pwm = int(right_pwm)
            command_str = f"L{left_pwm:03d}R{right_pwm:03d}\n"
            written = self.safe_write(command_str.encode())
            if written:
                print(f"SENT [{current_state}]: L{left_pwm:03d}R{right_pwm:03d} | Radar: {min_dist:.0f} | dt: {current_time - self.last_send_time:.3f}s")
                self.last_send_time = current_time

    def start_sweep(self):
        """Triggers the sensor sweep on Arduino."""
        command_str = "START_SWEEP\n"
        self.safe_write(command_str.encode())

    def stop_car(self):
        """Stops the car immediately by writing zero PWM values."""
        command_str = "L000R000\n"
        self.safe_write(command_str.encode())
        time.sleep(0.1)

    def close(self):
        """Cleans up and closes the serial port."""
        if self.ser and self.ser.is_open:
            self.stop_car()
            self.ser.close()
            print("✅ Serial port closed safely")
