# LEGO type:standard slot:0 autostart
from mindstorms import MSHub, Motor, DistanceSensor
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
#endregion

#region Functions
# Arc functions
# Arc functions can be visualised in ai-simulation/curve_sim
def hybrid_arc(angle):
    # use the lerp arc until certain angle behind the ball, then turn in a circle
    threshold, circle_turn = params.get("hybrid_cutoff")
    perc = abs(angle / pi)
    if perc > threshold: return lerp_arc(angle)
    return circle_turn

def lerp_arc(angle):
    # adjust turn factor with angle to the ball between the extremes in the settings
    front_turn, back_turn = params.get("lerp_arc_factors")
    perc = abs(angle / pi)
    return lerp(front_turn, back_turn, perc)


# Helpful Math Functions
def sign(x):
    if x == 0: return 0
    return int(x / abs(x))

def lerp(val, targ, amm):
    # Lerp is Linear intERPolation. Used for smoothing of values
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

    
class Norm:
    def square(velocity):
        # Square norm maximises the bot's speed, but makes it non uniform (fastest when moving forward, backward left and right, and slowest on diagonals)
        velocity = complex(velocity)

        max_coord = max(abs(velocity.real), abs(velocity.imag))
        if max_coord <= 1: return velocity

        return velocity / max_coord

    def circle(velocity):
        # circle norm makes the bot slower when moving forward/back/left/right, but it has a uniform speed for all directions.
        # Used on one bot so they have variation in AI and don't constantly clump together.
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
def screen(data, fps=14, loop=True):
    if type(data) is str:
        hub.light_matrix.show(data)
    else:
        hub.light_matrix.start_animation(data, fps, loop)

def bot_screen():
    if bot == DOT:
        return Screen.dot 
    return Screen.blu

def bot_loading():
    if bot == DOT:
        return Screen.dot_load
    return Screen.blu_load

class Screen:
    off = "00000:00000:00000:00000:00000"
    blu = "00000:09090:09090:00990:00000"
    dot = "00000:09990:09990:09990:00000"
    dot_shield = "00000:09090:09990:00900:00000"

    dot_load = ['70000:59990:09990:09990:00000', '57000:09990:09990:09990:00000', '05700:09990:09990:09990:00000', '00570:09990:09990:09990:00000', '00057:09990:09990:09990:00000', '00005:09997:09990:09990:00000', '00000:09995:09997:09990:00000', '00000:09990:09995:09997:00000', '00000:09990:09990:09995:00007', '00000:09990:09990:09990:00075', '00000:09990:09990:09990:00750', '00000:09990:09990:09990:07500', '00000:09990:09990:09990:75000', '00000:09990:09990:79990:50000', '00000:09990:79990:59990:00000', '00000:79990:59990:09990:00000']
    blu_load = ['70000:59090:09090:00990:00000', '57000:09090:09090:00990:00000', '05700:09090:09090:00990:00000', '00570:09090:09090:00990:00000', '00057:09090:09090:00990:00000', '00005:09097:09090:00990:00000', '00000:09095:09097:00990:00000', '00000:09090:09095:00997:00000', '00000:09090:09090:00995:00007', '00000:09090:09090:00990:00075', '00000:09090:09090:00990:00750', '00000:09090:09090:00990:07500', '00000:09090:09090:00990:75000', '00000:09090:09090:70990:50000', '00000:09090:79090:50990:00000', '00000:79090:59090:00990:00000']

#endregion

#region Params
# Bluetooth MAC adesses assigned to each hub. this is printed on progam load
# and is used by the program to identify wich bot is being used.
HUBS = [
    "38:0B:3C:AA:AA:CD", # Blu
    "A8:E2:C1:9A:7C:6B", # Dot
    "A8:E2:C1:95:24:D9"  # Spare Hub
]
BLU = HUBS[0]
DOT = HUBS[1]

SHOW_POSITION = True # weather to show an indicator of position on the screen, used for debugging

params = { # Settings
    # Measurements
    "field_size":        (182, 243),  # width and height of the soccer field (cm) used for calculating position
    "wheel_radius":      2.8,         # cm, used to calculate position

    "speed":             1,           # Percent of the bots max speed to move at
    "dot_speed":         .95,         # Dot's speed in Attacker mode

    # Finding the Goal
    "goal_align_zone":   radians(20), # zone in front of robot where it will center itself on the goal if the ball is there
    "goal_align_speed":  .003,        # strength of push toward the goal
    "goal_align_limit":  .1,          # maximum strength of goal push

    # Capturing the Ball
    "arc_function":      hybrid_arc,  # function to use when trying to get behind the ball
    "turn_deadspace":    radians(20), # angle range to consider the ball "captured" and start moving towards the goal
    "lerp_arc_factors":  (2, 1.3),    # (front, behind), extreme turn factors for the lerp arc function
    "hybrid_cutoff":     (.7, 1.75),  # (threshold, turn amm), for the hybrid function, threshold is when to switch to circular turning, and turn amm is how tightly to turn

    # Gyro (Compass/Rotation) correction
    "angle_deadzone":    radians(4),  # error margin for what's considered "facing forward" (set to ± 4°)
    "angle_correct":     .2,          # speed to rotate back to forward when knocked around (the robot sacrifices speed to rotate faster, so keep this small)
    "angle_interpolate": .8,          # how much to smooth out sensor readings, this makes the robot turn more smoothly. 1 = no smoothing, 0 = very slow smoothig (slow smoothing means the bot sees the ball where it was further in the past) (lerp speed of interpolaton)
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
        self.motor_angles = [0, 0, 0, 0]

        # reset motor counts
        for motor in self.motors:
            motor.set_stop_action("hold")

        self.reset_motors()

    def reset_motors(self):
        self.motor_angles = [0, 0, 0, 0]
        for motor in self.motors:
            motor.set_degrees_counted(0)

    def add(self, dir, speed=1):
        if dir == None: return
        # speed = clamp(speed, 0, 1) # this not needed?
        dir -= self.wheel_angle
        self.velocity += exp(complex(0, dir)) * speed

    def add_spin(self, spin):
        self.spin += spin
        self.spin = clamp(self.spin, -1, 1)

    def move(self):
        # apply queued up movement to motors. 
        # note: translational movement is slowed down to allow for rotation. if spin == 1 the robot will stop moving to spin at maximum speed

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
        # use motor turning measurements to calculate how far the robot has moved (keep track of position on the field)
        counts = []
        # motors change order (x, -x, y, -y) -> (x, x, y, y)
        for n, motor in enumerate(self.motors):
            counts.append(
                # flip every other motor count (they spin opposite)
                (-1)**n * radians(motor.get_degrees_counted())
            )

        delta_counts = [count - pcount for count, pcount in zip(counts, self.motor_angles)]

        # average both measusements for more accuracy
        motor_x = average(delta_counts[:2])
        motor_y = average(delta_counts[2:])

        # convert to cm
        wheel_radius = params.get("wheel_radius")
        motor_x *= wheel_radius
        motor_y *= wheel_radius

        # measurements are still offset from x axis
        # convert basis to align with robot
        motor_x *= exp(complex(0,      self.wheel_angle))
        motor_y *= exp(complex(0, pi - self.wheel_angle))

        delta_position = motor_x + motor_y

        # add to position if no lone spinning wheels
        self.position += delta_position
        self.motor_angles = counts

        # clamp position to field size to account for walls
        # this also helps to fix position errors when bot runs up against the wall
        field_half = params.get("field_size")[0] / 2
        self.position = complex(
            self.position.real, 
            clamp(self.position.imag, -field_half, field_half)
        )

        # display status
        # show side of field in position on screen
        if SHOW_POSITION:
            screen(bot_screen())
            x = 4 if sign(self.position.imag) == 1 else 0 # set this to a range distance from center?
            hubdata.display.pixel(x, 4, 9)

        return self.position

class SensorHandler:
    def __init__(self, distance_sensor, ir_segments=12):
        # make sure The DIP switches on the disk are configured correctly: ON OFF OFF
        self.ir_sensor = DistanceSensor(distance_sensor)

        self.ir_segments = ir_segments
        self.reset()

    def reset(self):
        hub.motion_sensor.reset_yaw_angle()
        self.lerp_bdir = self.ball_direction() or 0

        movement.position = complex(0)
        movement.reset_motors()

    # Ball
    def interpolate_dir(self):
        # smooth out the sensed ball angle, so the robot moves smoothly (helps maintain momentum)
        ball = self.ball_data()

        if ball.get("ball_found"):
            self.lerp_bdir = lerp(
                self.lerp_bdir, 
                ball.get("raw_angle"), 
                params.get("angle_interpolate")
            )

    def ir_dir(self):
        # returns the segment on the sensor where the ball is detected
        return self.ir_sensor.get_distance_cm()

    def ball_direction(self):
        # returns angle to the ball in radians, with 0 as forward.
        # make sure 0 is forward (for multiplication of angle)
        quadrant = self.ir_dir()
        if quadrant == self.ir_segments: 
            quadrant = 0

        angle = (2*pi/self.ir_segments) * quadrant # 1/12 of angle = 2pi/12

        if quadrant > self.ir_segments/2: 
            angle -= 2*pi # convert to range [-pi, pi)

        return -angle

    def ball_data(self):
        """Returns Interpreted Data about the ball's relative position"""
        ir_dir  = self.ir_dir()

        return {
            "raw_angle":    self.ball_direction(),
            "angle":        self.lerp_bdir,
            "ball_found":   ir_dir != 0 # TODO: needs check
        }

    # Motion 
    def rotation(self):
        return radians( hub.motion_sensor.get_yaw_angle() )

    def left_button(self):
        return hub.left_button.was_pressed()
    
    def right_button(self):
        return hub.right_button.was_pressed()
    
    def center_button(self):
        return hubdata.button.center.is_pressed()

    @staticmethod
    def battery_status():
        # useful for competitions when hub us under heavy use, swap out the hub before it gets too hot!
        temp    = hubdata.battery.temperature()
        charge  = hubdata.battery.capacity_left()
        current = hubdata.battery.current()
        voltage = hubdata.battery.voltage()

        print(hubdata.supervision.info())
        print("Battery:\n  Temperature: {}\n  Capacity Left: {}\n  Current: {}\n  Voltage: {}\n" \
            .format(temp, charge, current, voltage))

class Action:
    def correct_angle():
        # Rotates the robot so its facing the goal (yaw ≈ 0)
        rot = sensors.rotation()
        if abs(rot) > params.get("angle_deadzone"):
            movement.add_spin(rot * params.get("angle_correct"))

    def follow_ball():
        # blindly follow the ball (use for testing)
        movement.add(sensors.ball_data().get("angle"))

    def chase():
        # use the arc function to follow the ball (getting behind it first)
        ball = sensors.ball_data()
        angle = ball.get("angle")

        # display weather ball is detected on hub screen
        if not ball.get("ball_found"): 
            hubdata.display.pixel(2, 0, 0)
            return
        else:
            hubdata.display.pixel(2, 0, 9)

        if angle is None:  return
        
        # check for ball before moving
        if abs(angle) >= params.get("turn_deadspace"):
            turn_ammount = params.get("arc_function")(angle)
        else: turn_ammount = 0

        movement.add( angle * turn_ammount )

    def startup():
        global state, start_time, last_pos_check
        print("Running startup.")
        screen(bot_loading())

        while True:
            # buttons determine attack strategy used in the game.
            # pressing the right button on each bot will activate the "power build"
            # where each bot behaves identically, so they clump together and push the ball with more force.
            # this is useful when the opposition has strong or heavy bots.
            # pressing the left buttons activates the standard strategy where the bots have different speeds 
            # this allows dot to hang back and support from behind
            if sensors.right_button():
                state = AI.attacker
                break 

            if sensors.left_button():
                state = AI.attacker
                if bot == DOT: 
                    params["speed"] = params.get("dot_speed")

                # if bot == DOT:
                #     state = AI.goalie
                # else:
                #     state = AI.attacker
                break 

        # state = Action.drive_test

        screen(bot_screen())
        sensors.reset()
        start_time     = time_ns()
        last_pos_check = time_ns()

    def drive_test():
        # use to test if the wheels are configured correctly
        Action.correct_angle()
        movement.add(FORWARD)


    def goal_align():
        # push the bot horizontally toward the goal (assumed to be in front of its starting position)
        ball = sensors.ball_data()
        if not ball.get("ball_found"): return
        if ball.get("angle") is None:  return

        if abs(ball.get("angle")) < params.get("goal_align_zone"):
            align_speed = params.get("goal_align_speed") * movement.position.imag
            limit = params.get("goal_align_limit")
            align_speed = clamp(align_speed, -limit, limit)
            movement.add(RIGHT, align_speed)

    def back_push():
        # used to narrow the arc function so the bot doesn't hit the edges of the field.
        angle = sensors.ball_data().get("angle")
        if angle is None: return
        
        spread = pi/4
        if abs(angle) > pi - spread and abs(angle) < pi + spread:
            movement.add(BACKWARD, 2)

class AI:
    def attacker():
        global last_pos_check
        Action.chase()
        Action.correct_angle()

        movement.update_position()
        Action.goal_align()
        Action.back_push()

#region Init Program
hub      = MSHub()
movement = VectorMovementSystem("ADCB") # Ports where motors are connected in order (x, -x, y, -y)
sensors  = SensorHandler("E")           # Port where ball sensor is connected
bot      = Bluetooth.device_adress()

print(bot) # show the bot's MAC address to add to the HUBS list at the top

start_time     = 0
last_pos_check = 0

# Restart loop
# pressing center button stops the program, otherwise the bots can be started/stopped to move them on the field by hand
# this also resets the goal position and forward angle to wherever the bot is facing when its started again. (This is a huge advantage)
while not sensors.center_button(): 
    Action.startup()
    SensorHandler.battery_status()

    # In-game Loop
    while True:
        SensorHandler.battery_status()

        state()
        sensors.interpolate_dir()
        movement.move()

        if sensors.left_button() or sensors.right_button():
            movement.stop()
            break

#endregion
