# LEGO type:standard slot:0 autostart
# COM11 blu COM 15 Dot
from mindstorms import MSHub, Motor, ColorSensor
from time       import time_ns
from math       import radians
from cmath      import *
import ustruct as struct
import hub     as hubdata

#region Globals
LEFT     = pi/2
RIGHT    = -LEFT
FORWARD  = 0
BACKWARD = pi

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
def time_ms():
    return time_ns() / 1e6

def time_since(time):
    return time_ms() - time
#endregion

#region Screen
def screen(data):
    hub.light_matrix.show(data)

class Screen:
    alert   = "40904:40904:40904:00000:40904"
    amazed  = "09090:00000:09990:09990:09990"
    dead    = "09990:99999:90909:99999:09090"
    happy   = "09090:00000:97779:99999:09990"
    neutral = "00000:09090:00000:09990:00000"
    sqirm   = "00000:92029:29092:00000:09990"
    serious = "00000:90009:99099:00000:09990"
    off     = "00000:00000:00000:00000:00000"
    square  = "00000:09990:09990:09990:00000"

#endregion

#region Params
params = {
    "speed":             1,           # global speed
    "speed_norm":        Norm.square, # normilisation for final motor powers

    "turn_factor":       (2, 1.25),    # (front, behind) # (2, 1.15)?
    "turn_deadspace":    10,          # angle range that resets turning to 0

    "angle_deadzone":    4,           # deadzone size
    "angle_correct":     .38,         # speed of correction
    "angle_interpolate": .8,          # lerp speed of interpolaton

    "force_threshold":   800,         # size of a "significant force"
    "field_brightness":  (17, 40)     # range for brightness of the green carpet
}

angle_params = ["turn_deadspace", "angle_deadzone"]
for param in angle_params:
    params[param] = radians(params.get(param))
#endregion

class VectorMovementSystem:
    def __init__(self, motors: str, wheel_angle=45):
        """ motors in order: (x, -x, y, -y) """
        self.motors       = [Motor(port) for port in motors]
        self.velocity     = 0
        self.spin         = 0
        self.wheel_angle  = radians(wheel_angle)

        # reset motor counts
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
        norm = params.get("speed_norm")(self.velocity)
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
        global state
        screen(Screen.happy)

        # choose program type
        while True:
            if sensors.right_button():
                state = AI.attacker
                break 

            if sensors.left_button():
                # state = AI.goalie
                state = AI.attacker
                params["speed"] = .9
                break 

        screen(Screen.serious)
        sensors.reset()

    def intercept():
        if not sensors.on_line():
            # move horizontally to the ball's predicted location
            pass

class AI:
    def attacker():
        Action.chase()
        Action.correct_angle()

    def goalie():
        Action.intercept()
        Action.correct_angle()

#region Init Program
hub      = MSHub()
movement = VectorMovementSystem("BACD")
sensors  = SensorHandler("E", "F")

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
