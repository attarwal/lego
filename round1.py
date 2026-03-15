from hub import port
import runloop
import motor_pair
import distance_sensor

# --- Sensors ---
LEFT_SENSOR = port.F
RIGHT_SENSOR = port.A

# --- Units: distance_sensor returns mm ---
MAX_VALID_DISTANCE = 1800
STOP_DISTANCE_MM = 50        # 5 cm
FALLBACK_MAX_AGE = 10        # iterations for last-good reuse

# --- Console / dump ---
PRINT_EVERY = 10            # reduce spam
DUMP_LINE_DELAY_MS = 2        # throttle dump so lines aren't dropped

# =========================
# Helpers / Monitoring
# =========================
class SensorMonitor:
    def __init__(self, name):
        self.name = name
        self.last_value = -1
        self.stuck_count = 0
        self.invalid_count = 0
        self.STUCK_THRESHOLD = 3
        self.UNSTABLE_THRESHOLD = 5

    def check_reading(self, value):
        is_stuck_reading = (value >= MAX_VALID_DISTANCE) or (value < 0)
        if is_stuck_reading:
            self.invalid_count += 1
            if value == self.last_value or (value < 0 and self.last_value < 0):
                self.stuck_count += 1
            else:
                self.stuck_count = 1
        else:
            self.stuck_count = 0
            self.invalid_count = 0

        self.last_value = value
        return self.stuck_count >= self.STUCK_THRESHOLD and self.invalid_count >= self.UNSTABLE_THRESHOLD

    def is_unstable(self):
        return self.invalid_count >= 2 and self.stuck_count < self.STUCK_THRESHOLD

    def reset(self):
        self.stuck_count = 0
        self.invalid_count = 0
        self.last_value = -1

left_monitor = SensorMonitor("LEFT")
right_monitor = SensorMonitor("RIGHT")

class SensorValueTracker:
    def __init__(self):
        self.last_good_value = -1
        self.last_good_time = -999999

    def update(self, value, iteration):
        # IMPORTANT: allow 0 as "valid close"
        if 0 <= value < MAX_VALID_DISTANCE:
            self.last_good_value = value
            self.last_good_time = iteration
        return self.last_good_value

    def age(self, iteration):
        return iteration - self.last_good_time

left_tracker = SensorValueTracker()
right_tracker = SensorValueTracker()

def is_valid(v):
    return 0 <= v < MAX_VALID_DISTANCE

# =========================
# Wiggle Recovery
# =========================
async def wiggle_recovery(stuck_sensor_side):
    print("\n>>> WIGGLE RECOVERY for", stuck_sensor_side, "sensor <<<")

    if stuck_sensor_side == "left":
        first_steer, second_steer = 15, -15
        sensor = LEFT_SENSOR
    else:
        first_steer, second_steer = -15, 15
        sensor = RIGHT_SENSOR

    wiggle_steps = [
        (first_steer, 50, 200),
        (second_steer, 50, 200),
        (first_steer, 50, 200),
        (second_steer, 50, 200),
        (0, 40, 150),
    ]

    for step_num, (steering, velocity, duration) in enumerate(wiggle_steps):
        print("Wiggle step", step_num + 1, "steer=", steering, "vel=", velocity)
        motor_pair.move(motor_pair.PAIR_1, steering, velocity=velocity)
        await runloop.sleep_ms(duration)

        test = distance_sensor.distance(sensor)
        print("Test reading:", test)
        if is_valid(test):
            print(">>> Sensor recovered during wiggle! <<<\n")
            motor_pair.stop(motor_pair.PAIR_1)
            return True

    motor_pair.stop(motor_pair.PAIR_1)

    test = distance_sensor.distance(sensor)
    recovered = is_valid(test)
    if recovered:
        print(">>> Sensor recovered after wiggle! <<<\n")
    else:
        print(">>> Wiggle did not recover sensor <<<\n")
    return recovered

# =========================
# FAST emergency read (NO averaging)
# =========================
def fast_read_both():
    # no sleeps here — this is for emergency stop responsiveness
    l = distance_sensor.distance(LEFT_SENSOR)
    r = distance_sensor.distance(RIGHT_SENSOR)
    return l, r

# =========================
# Stable read (fast-ish median)
# Reduce delays so it doesn't slow the stop loop.
# =========================
async def get_both_sensors_stable(num_samples=3, delay_ms=6):
    left_readings = []
    right_readings = []

    for _ in range(num_samples):
        l = distance_sensor.distance(LEFT_SENSOR)
        if is_valid(l):
            left_readings.append(l)

        await runloop.sleep_ms(delay_ms)

        r = distance_sensor.distance(RIGHT_SENSOR)
        if is_valid(r):
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

# =========================
# Main
# =========================
async def main():
    motor_pair.pair(motor_pair.PAIR_1, port.D, port.C)

    # --- Motion setup ---
    distance_cm = 95
    wheel_circumference_cm = 17.5
    total_degrees = int((distance_cm / wheel_circumference_cm) * 360)

    velocity = 150
    sample_interval_ms = 15# MUCH faster loop for stop responsiveness

    travel_time_ms = int((total_degrees / velocity) * 1000)
    max_iters = max(1, travel_time_ms // sample_interval_ms)

    # Log: (i, fastL, fastR, stableL, stableR, usedL, usedR, status)
    log = []

    print("Starting move + collection...")
    print("Stop threshold:", STOP_DISTANCE_MM, "mm (5 cm)")
    print("Loop interval:", sample_interval_ms, "ms")
    print("Max iterations:", max_iters)
    print("")

    motor_pair.move(motor_pair.PAIR_1, 0, velocity=velocity)

    stopped_early = False

    for i in range(max_iters):

        # ====== 1) FAST STOP CHECK ======
        fastL, fastR = fast_read_both()

        # Treat fast valid readings (including 0) as usable for emergency stop
        fastL_ok = is_valid(fastL)
        fastR_ok = is_valid(fastR)

        if fastL_ok and fastR_ok:
            if fastL <= STOP_DISTANCE_MM and fastR <= STOP_DISTANCE_MM:
                motor_pair.stop(motor_pair.PAIR_1)
                await runloop.sleep_ms(0)
                motor_pair.stop(motor_pair.PAIR_1)# stop twice to be safe
                stopped_early = True
                print("!!! STOP (FAST): L=", fastL, "R=", fastR, "i=", i)
                log.append((i, fastL, fastR, -1, -1, fastL, fastR, "STOP_FAST"))
                break

        # ====== 2) STABLE READ (for logging & stuck detection) ======
        stableL, stableR = await get_both_sensors_stable(num_samples=3, delay_ms=6)

        # trackers update on stable values
        lastL = left_tracker.update(stableL, i)
        lastR = right_tracker.update(stableR, i)

        # stuck/unstable based on stable values
        left_stuck = left_monitor.check_reading(stableL)
        right_stuck = right_monitor.check_reading(stableR)
        left_unstable = left_monitor.is_unstable()
        right_unstable = right_monitor.is_unstable()

        # wiggle only if stuck
        if left_stuck:
            if await wiggle_recovery("left"):
                left_monitor.reset()
                stableL, stableR = await get_both_sensors_stable(num_samples=3, delay_ms=6)
                lastL = left_tracker.update(stableL, i)
                lastR = right_tracker.update(stableR, i)

        if right_stuck:
            if await wiggle_recovery("right"):
                right_monitor.reset()
                stableL, stableR = await get_both_sensors_stable(num_samples=3, delay_ms=6)
                lastL = left_tracker.update(stableL, i)
                lastR = right_tracker.update(stableR, i)

        # ====== 3) USED VALUES (fallback for minimizing skips/logging) ======
        usedL = stableL
        usedR = stableR
        status = "OK"

        if not is_valid(stableL):
            if lastL >= 0 and left_tracker.age(i) <= FALLBACK_MAX_AGE:
                usedL = lastL
                status = "FALLBACK_L"
            else:
                usedL = -1

        if not is_valid(stableR):
            if lastR >= 0 and right_tracker.age(i) <= FALLBACK_MAX_AGE:
                usedR = lastR
                status = "FALLBACK_R" if status == "OK" else status + "+FALLBACK_R"
            else:
                usedR = -1

        if usedL < 0 or usedR < 0:
            status = "NO_DATA"

        # occasional debug print
        if (i % PRINT_EVERY) == 0:
            ls = "[STUCK]" if left_stuck else ("[UNSTABLE]" if left_unstable else "")
            rs = "[STUCK]" if right_stuck else ("[UNSTABLE]" if right_unstable else "")
            print("i=", i,
                "fastL=", fastL, "fastR=", fastR,
                "| stableL=", stableL, ls,
                "stableR=", stableR, rs,
                "| usedL=", usedL, "usedR=", usedR,
                "|", status)
            await runloop.sleep_ms(0)

        log.append((i, fastL, fastR, stableL, stableR, usedL, usedR, status))

        await runloop.sleep_ms(sample_interval_ms)

    motor_pair.stop(motor_pair.PAIR_1)

    print("")
    print("Run complete. Stopped early?", stopped_early)
    print("Rows logged:", len(log))

    # ====== CSV DUMP ======
    print("")
    print("=== CSV DATA DUMP ===")
    print("i,fastL,fastR,stableL,stableR,usedL,usedR,status")

    for row in log:
        i, fastL, fastR, stableL, stableR, usedL, usedR, status = row
        print(str(i) + "," + str(fastL) + "," + str(fastR) + "," +
            str(stableL) + "," + str(stableR) + "," +
            str(usedL) + "," + str(usedR) + "," + status)
        await runloop.sleep_ms(DUMP_LINE_DELAY_MS)

    print("Dump complete.")

runloop.run(main())
