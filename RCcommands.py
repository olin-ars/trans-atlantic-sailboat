from evdev import InputDevice, categorize, ecodes
gamepad = InputDevice('/dev/input/event17')

sailDownMax = 0
sailUpMax = 1700
rudderLeftMax = 170
rudderRightMax = 1478
rudderRight = 180
rudderLeft = -180
sailOut = 180
sailIn = -180

directions = ['SailLR', 'SailUD', 'RudderLR', 'RudderUD']

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
            print(directions[event.code], position)
        #elif event.code == 0:
        #    sailPos = translate(event.value, 1874, 548, sailIn, sailOut)
        #    print(directions[event.code], sailPos)
        elif event.code == 2:
            position = translate(event.value, rudderLeftMax, rudderRightMax, rudderLeft, rudderRight)
            motor = 2
            print(directions[event.code], position)
        #elif event.code == 3:
        #    rudderPos = translate(event.value, 170, 1500, rudderLeft, rudderRight)
        #    print(directions[event.code], rudderPos)
