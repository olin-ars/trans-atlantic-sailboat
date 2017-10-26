from evdev import InputDevice, categorize, ecodes
gamepad = InputDevice('/dev/input/event17')

sailDownMax = 0
sailUpMax = 1700
rudderLeftMax = 170
rudderRightMax = 1478

directions = ['SailLR', 'SailUD', 'RudderLR', 'RudderUD']

for event in gamepad.read_loop():
    print(event.code, event.value)
