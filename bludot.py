# LEGO type:standard slot:0 autostart
from mindstorms import MSHub, Motor, ColorSensor
from time       import time_ns
from math       import radians
from cmath      import *
import hub as hubdata

#region Globals
LEFT     = pi/2
RIGHT    = -LEFT
FORWARD  = 0
BACKWARD = pi

SECOND = 1e9

dataports = {
    "A": hubdata.port.A, "B": hubdata.port.B,
    "C": hubdata.port.C, "D": hubdata.port.D,
    "E": hubdata.port.E, "F": hubdata.port.F,
}
#endregion

#region Functions
def sign(x):
    if x == 0: return 0
    return int(x / abs(x))

def lerp(val, targ, amm):
    if val is not None:
        return (targ - val) * amm + val
    else: return None

def clamp(x, minimum, maximum):
    return max(min(x, maximum), minimum)

def average(*args):
    """*args: average of a list object or of the arguments"""
    # if first argument is not a number type
    if not isinstance(args[0], (int, float, complex)): args = args[0]
    return sum(args) / len(args)

def aprox_equal(a, b, error_margin):
    return b - error_margin < a < b + error_margin
    
class Norm:
    def square(velocity):
        velocity = complex(velocity)

        max_coord = max(abs(velocity.real), abs(velocity.imag))
        if max_coord <= 1: return velocity

        return velocity / max_coord

    def circle(velocity):
        velocity = complex(velocity)
        if velocity == 0: return velocity
        
        magnitude = abs(velocity)

        if magnitude < 0:
            return velocity
        else: return velocity / magnitude

# Time Functions
def time_since(time):
    return time_ns() - time
#endregion

#region Screen
def screen(data):
    hub.light_matrix.show(data)

def default_screen():
    if bot == DOT:
        return Screen.dot 
    return Screen.blu

class Screen:
    alert   = "40904:40904:40904:00000:40904"
    amazed  = "09090:00000:09990:09990:09990"
    dead    = "09990:99999:90909:99999:09090"
    happy   = "09090:00000:97779:99999:09990"
    neutral = "00000:09090:00000:09990:00000"
    sqirm   = "00000:92029:29092:00000:09990"
    serious = "00000:90009:99099:00000:09990"
    off     = "00000:00000:00000:00000:00000"

    dot = "00000:08980:09990:08980:00000"
    blu = "99099:99099:99099:99999:09998"

#endregion

#region Params
BLU = ""
DOT = "A8:E2:C1:9A:7C:6B"

params = {
    "speed":             1,
    "dot_speed":         .9,          # Dot's speed in Attacker mode

    "wheel_radius":      3.175,       # cm
    "wheel_error":       radians(30), # min wheel difference to not be counted as moving

    "turn_factor":       (2, 1.25),   # (front, behind)
    "turn_deadspace":    radians(10), # angle range that resets turning to 0

    "angle_deadzone":    radians(4),  # deadzone size
    "angle_correct":     .38,         # speed of correction
    "angle_interpolate": .8,          # lerp speed of interpolaton

    "force_threshold":   800,         # size of a "significant force"
    "field_brightness":  (17, 40)     # range for brightness of the green carpet
}
#endregion

class Bluetooth:
    @staticmethod
    def device_adress():
        return hubdata.bluetooth.info().get("mac_addr")

class VectorMovementSystem:
    def __init__(self, motors: str, wheel_angle=45):
        """ motors in order: (x, -x, y, -y) """
        self.motors       = [Motor(port) for port in motors]
        self.velocity     = 0
        self.spin         = 0
        self.wheel_angle  = radians(wheel_angle)
        self.position     = 0

        # reset motor counts
        self.reset_motors()

    def reset_motors(self):
        for motor in self.motors:
            motor.set_stop_action("hold")

    def add_velocity(self, dir, speed=1):
        if dir == None: return
        # speed = clamp(speed, 0, 1) # this not needed?
        dir -= self.wheel_angle
        self.velocity += exp(complex(0, dir)) * speed

    def add_spin(self, spin):
        self.spin += spin
        self.spin = clamp(self.spin, -1, 1)

    def move(self):
        norm = Norm.square(self.velocity)
        dx = norm.real
        dy = norm.imag

        for motor, power in zip(self.motors, (dx, -dx, dy, -dy)):
            # scale down to make room for spinning,
            # then blend spinning into movement using that extra space
            power_scale = 1 - abs(self.spin)
            power_comp  = power * power_scale + self.spin

            motor.start(int(100 * power_comp * params.get("speed")))

        self.velocity = 0
        self.spin     = 0

    def stop(self):
        for motor in self.motors:
            motor.stop()

    def update_position(self):
        counts = []
        # motors change order (x, -x, y, -y) -> (x, x, y, y)
        for n, motor in enumerate(self.motors):
            counts.append(
                # flip every other motor count (they spin opposite)
                (-1)**n * radians(motor.get_degrees_counted())
            )

        # average both measusements for more accuracy
        motor_x = average(counts[:2])
        motor_y = average(counts[2:])

        # convert to cm
        wheel_radius = params.get("wheel_radius")
        motor_x *= wheel_radius
        motor_y *= wheel_radius

        # measurements are still offset from x axis
        # convert basis to align with robot
        motor_x *= exp(complex(0,      self.wheel_angle))
        motor_y *= exp(complex(0, pi - self.wheel_angle))

        delta_position = motor_x + motor_y

        self.reset_motors()

        # TODO: Check for consistency with accelerometer movement?
        
        # add to position if no lone spinning wheels
        x1, x2, y1, y2 = counts
        error_margin = params.get("wheel_error")
        if aprox_equal(x1, x2, error_margin) and aprox_equal(y1, y2, error_margin):
            self.position += delta_position

        return self.position

class SensorHandler:
    def __init__(self, distance_sensor: str, color_sensor: str, ir_segments=12):
        self.color = ColorSensor(color_sensor)

        # make sure The DIP switches on the disk are configured correctly: ON ON OFF
        self.ir_sensor = dataports.get(distance_sensor).device
        self.ir_sensor.mode(5, bytes([0,0,0,0])) # set to appropriate mode

        self.ir_segments = ir_segments
        self.reset()

    def reset(self):
        hub.motion_sensor.reset_yaw_angle()
        self.lerp_bdir = self.ball_direction(self.ir_data())

    # Ball
    def interpolate_dir(self):
        ball = self.ball_data()

        if ball.get("ball_found"):
            self.lerp_bdir = lerp(
                self.lerp_bdir, 
                ball.get("raw_angle"), 
                params.get("angle_interpolate")
            )

    def ir_data(self):
        data = self.ir_sensor.get()

        direction       = data[0]
        signal_strength = data[2]

        return {
            "quadrant":         direction,
            "signal_strength":  signal_strength # at least proportional if not equal, to distance to ball
        }

    def ball_direction(self, ir_data):
        if ir_data.get("signal_strength") == 0: return None

        # make sure 0 is forward (for multiplication of angle)
        quadrant = ir_data.get("quadrant")
        if quadrant == self.ir_segments: quadrant = 0

        # 1/12 of angle = 2pi/12
        angle = (2*pi/self.ir_segments) * quadrant

        # convert to range [-pi, pi)
        if quadrant > self.ir_segments/2: angle -= 2*pi
        return -angle

    def ball_data(self):
        """Returns Interpreted Data about the ball's relative position"""
        ir_data  = self.ir_data()
        strength = ir_data.get("signal_strength")

        # Convert Strength to distance in cm
        if strength != 0:
            distance = 192_000 / (strength**2)
        else:
            distance = None

        return {
            "raw_angle":    self.ball_direction(ir_data),
            "angle":        self.lerp_bdir,
            "strength":     strength,
            "distance":     distance,
            "ball_found":   strength != 0
        }

    # Motion 
    def rotation(self):
        return radians( hub.motion_sensor.get_yaw_angle() )

    def acceleration(self):
        """Returns: acceleration(x, y, z)"""
        x, z, y = hubdata.motion.accelerometer()
        # Y is flipped because gravity (downward) is read as positive
        return -x, -y, z # x is flipped in reading for some reason

    # Environment
    def brightness(self):
        return self.color.get_reflected_light()

    def on_line(self):
        floor_min, floor_max = params.get("field_brightness")
        on_line = not ( floor_min < sensors.brightness() < floor_max )
        return on_line 

    def left_button(self):
        return hub.left_button.was_pressed()
    
    def right_button(self):
        return hub.right_button.was_pressed()

class Action:
    def correct_angle():
        # Rotates the robot so the yaw is ~0
        rot = sensors.rotation()
        if abs(rot) > params.get("angle_deadzone"):
            movement.add_spin(rot * params.get("angle_correct"))

    def follow_ball():
        movement.add_velocity(sensors.ball_data().get("angle"))

    def chase():
        ball = sensors.ball_data()
        
        # check for ball before moving
        if ball.get("ball_found") and ball.get("angle") is not None:
            # ajust angle dynamically, bigger factor = wider curves,
            # shrink it when ball is behind to make the curve so its more compact
            # zero when its close to in front, so it doesn't amplify errors in the angle
            if abs(ball.get("angle")) >= params.get("turn_deadspace"):
                front_turn, back_turn = params.get("turn_factor")

                turn_ammount = abs(ball.get("angle")) / pi
                turn_ammount = lerp(front_turn, back_turn, turn_ammount)
            else: turn_ammount = 0

            movement.add_velocity(
                ball.get("angle") * turn_ammount
            )

    def startup():
        global state, start_time, last_pos_check
        screen(default_screen())

        while True:
            if sensors.right_button():
                # state = AI.test
                state = AI.attacker
                break 

            if sensors.left_button():
                # state = AI.test
                state = AI.attacker

                # if bot == DOT:
                #     state = AI.goalie
                # else:
                #     state = AI.attacker
                break 

        sensors.reset()
        start_time = time_ns()
        last_pos_check = time_ns()

    def intercept():
        if not sensors.on_line():
            # move horizontally to the ball's predicted location
            pass

class AI:
    def attacker():
        if bot == DOT: params["speed"] = params.get("dot_speed")
        Action.chase()
        Action.correct_angle()

    def goalie():
        Action.intercept()
        Action.correct_angle()

    def test():
        global start_time, last_pos_check
        if time_since(start_time) < SECOND:
            movement.add_velocity(0)

        if time_since(last_pos_check) > SECOND / 3:
            last_pos_check = time_ns()
            movement.update_position()
            print(movement.position)

#region Init Program
hub      = MSHub()
movement = VectorMovementSystem("BACD")
sensors  = SensorHandler("E", "F")
bot      = Bluetooth.device_adress()

start_time = 0
last_pos_check = 0

# Restart loop
while True:
    Action.startup()

    # In-game Loop
    while True:
        state()
        sensors.interpolate_dir()
        movement.move()

        if sensors.left_button() or sensors.right_button():
            movement.stop()
            break

#endregion
