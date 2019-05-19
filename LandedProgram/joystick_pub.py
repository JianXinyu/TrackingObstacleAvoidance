import pygame
from msgdev import MsgDevice,PeriodTimer
speed_limit = 1200
#Debug = False
Debug = True
pygame.init()
pygame.joystick.init()
class AbortProgram(Exception):
    pass
# -------- Main Program Loop -----------
if __name__ == "__main__":
    dev = MsgDevice()
    dev.open()
    dev.pub_bind('tcp://0.0.0.0:55001')
    try:
        t = PeriodTimer(0.1)
        t.start()
        while True:
            with t:
                # EVENT PROCESSING STEP
                for event in pygame.event.get(): # User did something       
                    # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
                    if event.type == pygame.JOYBUTTONDOWN:
                        print("Joystick button pressed.")
                    if event.type == pygame.JOYBUTTONUP:
                        print("Joystick button released.")
                
                for i in range(0,pygame.joystick.get_count()):
                    Joystick1 = pygame.joystick.Joystick(i)
                    if Joystick1.get_name() == "Mad Catz F.L.Y.5 Stick":
                        joystick=pygame.joystick.Joystick(i)
                joystick.init()        
                # joystick data
                diff = joystick.get_axis( 0 )
                if abs(diff) < 0.1:
                    diff = 0
                aver = joystick.get_axis( 1 )
                if abs(aver) < 0.1:
                    aver = 0
                state = joystick.get_axis( 2 )

                #processed data
                averspeed = aver*speed_limit*(-1)
                diffspeed = diff*speed_limit*0.5
                left_motor = (averspeed + diffspeed)*(-1)
                right_motor = averspeed - diffspeed
                if left_motor > speed_limit:
                    left_motor = speed_limit
                if left_motor < -speed_limit:
                    left_motor = -speed_limit
                if right_motor > speed_limit:
                    right_motor = speed_limit
                if right_motor < -speed_limit:
                    right_motor = -speed_limit
                if state >= 0:
                    autoctrl = 1
                if state < 0:
                    autoctrl = 0
                dev.pub_set1('js.left.speed',left_motor)
                dev.pub_set1('js.right.speed',right_motor)
                dev.pub_set1('js.autoctrl',autoctrl)
                if Debug == True:
                    print("left motor speed is",left_motor)
                    print ("right motor speed is",right_motor)
                    print ("auto-control state is",autoctrl)   
    except (Exception, KeyboardInterrupt, AbortProgram) as e:
        pygame.quit()
        dev.close()

