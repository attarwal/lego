# === Sensor Test with Wiggle Recovery + Stability Improvements ===
from hub import port
import runloop
import motor_pair
import distance_sensor

LEFT_SENSOR = port.F
RIGHT_SENSOR = port.A
MAX_VALID_DISTANCE = 1800

# === Sensor Monitor Class ===
class SensorMonitor:
    def __init__(self, name):
        self.name = name
        self.last_value = -1
        self.stuck_count = 0
        self.invalid_count = 0
        self.STUCK_THRESHOLD = 3
        self.UNSTABLE_THRESHOLD = 5# Allow some invalid readings before declaring stuck

    def check_reading(self, value):
        """Returns True if sensor appears stuck (consistently invalid or maxed out)"""
        is_stuck_reading = (value >= MAX_VALID_DISTANCE) or (value < 0)

        if is_stuck_reading:
            self.invalid_count += 1
            # Only consider it "stuck" if we have STUCK_THRESHOLD consecutive invalid readings
            if value == self.last_value or (value < 0 and self.last_value < 0):
                self.stuck_count += 1
            else:
                self.stuck_count = 1
        else:
            # Valid reading - reset counters
            self.stuck_count = 0
            self.invalid_count = 0

        self.last_value = value

        # Only trigger stuck if we have enough consecutive invalid readings
        # This prevents false positives when sensor is just at a difficult angle
        return self.stuck_count >= self.STUCK_THRESHOLD and self.invalid_count >= self.UNSTABLE_THRESHOLD

    def is_unstable(self):
        """Returns True if sensor is giving intermittent readings (not fully stuck)"""
        return self.invalid_count >= 2 and self.stuck_count < self.STUCK_THRESHOLD

    def reset(self):
        self.stuck_count = 0
        self.invalid_count = 0
        self.last_value = -1

left_monitor = SensorMonitor("LEFT")
right_monitor = SensorMonitor("RIGHT")

# === Wiggle Recovery ===
async def wiggle_recovery(stuck_sensor_side):
    """
    Move the car with small, minimal-speed movements to change the angle and unstick the sensor.
    """
    print("\n>>> WIGGLE RECOVERY for", stuck_sensor_side, "sensor <<<")

    if stuck_sensor_side == "left":
        first_steer = 15
        second_steer = -15
        sensor = LEFT_SENSOR
    else:
        first_steer = -15
        second_steer = 15
        sensor = RIGHT_SENSOR

    # Very small, slow wiggle movements (minimal velocity)
    wiggle_steps = [
        (first_steer, 50, 1200),
        (second_steer, 50, 1200),
        (first_steer, 50, 1200),
        (second_steer, 50, 1200),
        (0, 40, 1150),
    ]

    for step_num, (steering, velocity, duration) in enumerate(wiggle_steps):
        print("Wiggle step", step_num + 1, "steer=", steering, "vel=", velocity)
        motor_pair.move(motor_pair.PAIR_1, steering, velocity=velocity)
        await runloop.sleep_ms(duration)

        test = distance_sensor.distance(sensor)
        print("Test reading:", test)
        if 0 < test < MAX_VALID_DISTANCE:
            print(">>> Sensor recovered during wiggle! <<<\n")
            motor_pair.stop(motor_pair.PAIR_1)
            return True

    motor_pair.stop(motor_pair.PAIR_1)

    test = distance_sensor.distance(sensor)
    recovered = 0 < test < MAX_VALID_DISTANCE
    if recovered:
        print(">>> Sensor recovered after wiggle! <<<\n")
    else:
        print(">>> Wiggle did not recover sensor <<<\n")
    return recovered

# === Sensor Reading with Fallback ===
class SensorValueTracker:
    """Track last known good value for a sensor"""
    def __init__(self):
        self.last_good_value = -1
        self.last_good_time = 0

    def update(self, value, iteration):
        if 0 < value < MAX_VALID_DISTANCE:
            self.last_good_value = value
            self.last_good_time = iteration
        return self.last_good_value

left_tracker = SensorValueTracker()
right_tracker = SensorValueTracker()

async def get_both_sensors_stable(num_samples=5, delay_ms=30):
    """Read both sensors with staggered timing"""
    left_readings = []
    right_readings = []

    for _ in range(num_samples):
        l = distance_sensor.distance(LEFT_SENSOR)
        if 0 < l < MAX_VALID_DISTANCE:
            left_readings.append(l)

        await runloop.sleep_ms(delay_ms)

        r = distance_sensor.distance(RIGHT_SENSOR)
        if 0 < r < MAX_VALID_DISTANCE:
            right_readings.append(r)

        await runloop.sleep_ms(delay_ms)

    left_dist = -1
    right_dist = -1

    if left_readings:
        left_readings.sort()
        left_dist = left_readings[len(left_readings) // 2]

    if right_readings:
        right_readings.sort()
        right_dist = right_readings[len(right_readings) // 2]

    return left_dist, right_dist

def compute_eigenspace_coordinates(left_dist, right_dist):
    pc1_v1, pc1_v2 = 0.7370, -0.6759
    pc2_v1, pc2_v2 = 0.6759, 0.7370
    alpha_1 = pc1_v1 * (left_dist-30) + pc1_v2 * (right_dist-30)
    alpha_2 = pc2_v1 * (left_dist-30) + pc2_v2 * (right_dist-30)
    return alpha_1, alpha_2

# === Main Loop ===
async def main():
    motor_pair.pair(motor_pair.PAIR_1, port.D, port.C)
    num_iterations = 100
    sample_interval_ms = 200

    for i in range(num_iterations):
        left_dist, right_dist = await get_both_sensors_stable(num_samples=5, delay_ms=25)

        # Track last good values
        left_tracked = left_tracker.update(left_dist, i)
        right_tracked = right_tracker.update(right_dist, i)

        # Check for stuck sensors
        left_stuck = left_monitor.check_reading(left_dist)
        right_stuck = right_monitor.check_reading(right_dist)

        left_unstable = left_monitor.is_unstable()
        right_unstable = right_monitor.is_unstable()

        # Display with status indicators
        left_status = ""
        right_status = ""

        if left_stuck:
            left_status = "[STUCK]"
        elif left_unstable:
            left_status = "[UNSTABLE]"

        if right_stuck:
            right_status = "[STUCK]"
        elif right_unstable:
            right_status = "[UNSTABLE]"

        print("i=", i,
            "L=", left_dist, left_status, "(cnt:", left_monitor.stuck_count, ")",
            "R=", right_dist, right_status, "(cnt:", right_monitor.stuck_count, ")")

        # Only wiggle if truly stuck (not just unstable)
        if left_stuck:
            print("LEFT SENSOR STUCK - triggering wiggle")
            if await wiggle_recovery("left"):
                left_monitor.reset()
                left_dist, right_dist = await get_both_sensors_stable(num_samples=5, delay_ms=25)

        if right_stuck:
            print("RIGHT SENSOR STUCK - triggering wiggle")
            if await wiggle_recovery("right"):
                right_monitor.reset()
                left_dist, right_dist = await get_both_sensors_stable(num_samples=5, delay_ms=25)

        # Display readings with eigenspace coordinates if both valid
        left_valid = 0 < left_dist < MAX_VALID_DISTANCE
        right_valid = 0 < right_dist < MAX_VALID_DISTANCE

        if left_valid and right_valid:
            alpha_1, alpha_2 = compute_eigenspace_coordinates(left_dist, right_dist)
            alpha_1=abs(alpha_1)
            motor_pair.move(motor_pair.PAIR_1, int(alpha_2/10), velocity=int(alpha_1*.5))
            diff = left_dist - right_dist
            print("-> diff=", diff, "a1=", round(alpha_1, 1), "a2=", round(alpha_2, 1))
        elif left_valid or right_valid:
            # At least one is valid - show what we have
            if left_unstable or right_unstable:
                print("-> Using last good values: L=", left_tracked, "R=", right_tracked)

        await runloop.sleep_ms(sample_interval_ms)

    print("\n=== Test Complete ===")

runloop.run(main())
