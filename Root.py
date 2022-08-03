from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3

robot = Root(Bluetooth())

@event(robot.when_play)
async def play(robot):
    await robot.set_lights_rgb(255,115,0)
    print('Begin')
    await robot.move(20)
    
    await robot.set_lights_rgb(0,0,255)
    print('Moving Straight')
    await robot.move(40)
    
    await robot.set_lights_rgb(255,0,0)
    print('Turning')
    await robot.arc(Robot.DIR_RIGHT, 90, 0)
    
    await robot.set_lights_rgb(0,0,255)
    print('Moving Straight')
    await robot.move(20)

    await robot.set_lights_rgb(255,0,0)
    await robot.arc(Robot.DIR_RIGHT, 90, 0)

    await robot.set_lights_rgb(0,0,255)
    print('Moving Straight')
    await robot.move(40)

    await robot.set_lights_rgb(255,0,0)
    print('Turning')
    await robot.arc(Robot.DIR_RIGHT, 90, 0)

    await robot.set_lights_rgb(0,0,255)
    print('Moving Straight')
    await robot.move(20)

    await robot.set_lights_rgb(255,0,0)
    print('Turning')
    await robot.arc(Robot.DIR_LEFT, 90, 0)

    await robot.set_lights_rgb(255,115,0)
    print('Moving Straight')
    await robot.move(25)

    await robot.set_lights_rgb(0,255,0)
    print('Done')

@event(robot.when_bumped, [])
async def bumped(robot):
    await robot.set_lights_rgb(255,255,255)
    print('Process cancelled')
    await robot.move(0)

robot.play()
