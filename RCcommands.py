from evdev import InputDevice, categorize, ecodes
from dynamixels.dynamixel_controller import DynamixelController

gamepad = InputDevice('/dev/input/by-id/usb-Horizon_Hobby_SPEKTRUM_RECEIVER_00000000001A-event-joystick')

sailDownMax = 0
sailUpMax = 1700
rudderLeftMax = 170
rudderRightMax = 1478
rudderRight = 1024
rudderLeft = -1024
sailOut = 4000
sailIn = -4000

# Top-left 3-point toggle switch
TL_TOGG = 4
TOGG_DOWN = 213
TOGG_CENTER = 149
TOGG_UP = 42

directions = ['SailLR', 'SailUD', 'RudderLR', 'RudderUD']

main = DynamixelController(DynamixelController.MAIN)
jib = DynamixelController(DynamixelController.JIB)
rudder = DynamixelController(DynamixelController.RUDDER)

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

for event in gamepad.read_loop():
    if not event.type == 0:
        if event.code == 1:
            position = translate(event.value, sailDownMax, sailUpMax, sailIn, sailOut)
            motor = 0
            # print(directions[event.code], position)
            main.set_position(int(position))
            jib.set_position(int(position))
        elif event.code == 2:
            position = translate(event.value, rudderLeftMax, rudderRightMax, rudderLeft, rudderRight)
            motor = 2
            # print(directions[event.code], position)
            rudder.set_position(int(position))
        # elif event.code == TL_TOGG:  # Use top-left toggle switch for torque control
        #     if event.value == TOGG_DOWN:
        #         main.enable_torque()
        #         rudder.enable_torque()
        #     else:
        #         main.disable_torque()
        #         rudder.disable_torque()