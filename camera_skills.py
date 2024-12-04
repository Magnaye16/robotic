from vex import *


# Brain should be defined by default
brain=Brain()


# Robot configuration code
controller_1 = Controller(PRIMARY)

left_motor_a = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
left_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, False)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT15, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 319.19, 317.5, 300, MM, 0.2857142857142857)

conveyor_feeder_motor_a = Motor(Ports.PORT18, GearSetting.RATIO_18_1, False)
conveyor_feeder_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
feeder_motor = Motor(Ports.PORT7,GearSetting.RATIO_18_1,False)
conveyor_feeder = MotorGroup(conveyor_feeder_motor_a)

dispenser = Motor(Ports.PORT19,GearSetting.RATIO_18_1,False)
left_motor_arm = Motor(Ports.PORT20,GearSetting.RATIO_18_1,False)
right_motor_arm = Motor(Ports.PORT16,GearSetting.RATIO_18_1,True)
arm = MotorGroup(left_motor_arm,right_motor_arm)

BLUE_RING_sig = Signature(1, -4393, -2133, -3263,-1, 4567, 2283,1.1, 0)
RED_RING_sig = Signature(2, 6539, 10627, 8583,-2007, -1013, -1510,2.5, 0)
STAKE_sig = Signature(3, -2299, -201, -1250,-7495, -5343, -6419,2.5, 0)
front_vision = Vision(Ports.PORT6, 50, BLUE_RING_sig, RED_RING_sig, STAKE_sig)
back_vision = Vision(Ports.PORT3, 50, BLUE_RING_sig, RED_RING_sig, STAKE_sig)
middle_vision = Vision(Ports.PORT2, 50, BLUE_RING_sig, RED_RING_sig, STAKE_sig)
flapper = Motor(Ports.PORT10,GearSetting.RATIO_18_1,False)




grabber = Pneumatics(brain.three_wire_port.a)
aliance_ring = RED_RING_sig
stake = STAKE_sig

# wait for rotation sensor to fully initialize
wait(30, MSEC)


#region custom classes


def req_vel_to_center(obj_sign:Signature = aliance_ring,damp:float = 0.1)->float:
    vision_centerx =  (316/2)


    obj = nearest_obj(obj_sign)


    if obj is None :return 0
    return ( obj.centerX - vision_centerx ) * damp


#endregion custom classes



def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)


# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")
#endregion VEXcode Generated Robot Configuration


#region custom_drive_utils

def req_vel_to_center(obj_sign:Signature = aliance_ring,vision:Vision = front_vision,damp:float = 0.3)->float:
    vision_centerx =  (316/2)


    obj = nearest_obj(obj_sign,vision)


    if obj is None :return 0
    return ( obj.centerX - vision_centerx ) * damp

def search_obj(obj_sig:Signature,vision:Vision,search_dir:DirectionType = LEFT):
   
    while nearest_obj(obj_sig,vision) is None:
        drive_turn(5,search_dir)

looking = False
def look_at(obj_sign:Signature,vision:Vision = front_vision):
    global looking
    
    counter=0
    vel = req_vel_to_center(obj_sign,vision)
   
    print(vel)

    looking = True
    
    while looking :
        vel = req_vel_to_center(obj_sign,vision,0.1)
            #max tries     #prevent false centered
        if (abs(vel) < 0.5 and counter > 10) : break

        drivetrain_movement(vel)
        counter+=1
        wait(10,MSEC)
    
    looking = False
    drivetrain.stop()
 
def flapper_on():
        flapper.spin(FORWARD)
        at_max(flapper,lambda:flapper.stop)
def flapper_off():
        flapper.spin(REVERSE,10000)
        at_max(flapper,lambda:flapper.stop)
     
def at_max(motor:Motor,callback):

    wait(200,MSEC)

    while not motor.is_spinning():
        print(abs(motor.velocity()),"  ",motor.is_spinning())
        if not abs(motor.velocity()) > 0:
            print("maxed")
            callback()  
            break
        wait(10,MSEC)

def opposing_ring()->Signature:
    return RED_RING_sig if aliance_ring == BLUE_RING_sig else BLUE_RING_sig

def is_in_vision(obj_sig:Signature = aliance_ring,vision_cam:Vision=front_vision)->bool:
    if not vision_cam.installed():raise Exception('vision camera is not installed')
    
    objs = vision_cam.take_snapshot(obj_sig,1)
    return objs and len(objs) > 0

def nearest_obj(obj_sig:Signature = aliance_ring,vision_cam:Vision=front_vision)->VisionObject|None:
    if not is_in_vision(obj_sig,vision_cam):return None
    return vision_cam.largest_object()

def drive_turn(deg:float,dir:DirectionType = LEFT,vel:float=100):
    adjustment = 0.15
    drivetrain.turn_for(dir,deg * adjustment,DEGREES,vel)
   
def left_turn(deg):
    drive_turn(deg,LEFT)  

def right_turn(deg):
    drive_turn(deg,RIGHT)    

def drive_for(dist:float,velocity:float = 10,_wait=True):
    adjustment = 0.25
    drivetrain.drive_for(FORWARD,dist * adjustment,INCHES,velocity,PERCENT,_wait)
    if not _wait : wait(70,MSEC)

def drivetrain_movement(side_speed=0,forward_speed=0,from_controller:bool = False,speed_mul = 1):

    global looking
    
    if from_controller:
        
        if abs(controller_1.axis1.position() + controller_1.axis3.position()) <= 0 and looking :
            
            print("auto")

            return
        
        print("drivers control")
        
        side_speed = controller_1.axis1.position()
        forward_speed = controller_1.axis3.position()  


    target_right_drive_vel = (forward_speed - side_speed) * speed_mul
    target_left_drive_vel = (forward_speed + side_speed)* speed_mul
    
    right_drive_smart.spin(FORWARD,target_right_drive_vel,PERCENT)
    left_drive_smart.spin(FORWARD,target_left_drive_vel,PERCENT)

def conveyor_feed(feed:bool = True,out = False):
    if feed:return conveyor_feeder.spin(REVERSE if not out else FORWARD,100,PERCENT)
    conveyor_feeder.stop()

def feeder(feed:bool = True,out = False):
    if feed:return feeder_motor.spin(REVERSE if not out else FORWARD,100,PERCENT)
    feeder_motor.stop()

def intake(run:bool = True):
    feeder(run)
    conveyor_feed(run)

def conveyor_control():
    if controller_1.buttonR1.pressing():
            conveyor_feeder.spin(REVERSE,100,PERCENT)
            feeder_motor.spin(REVERSE,100,PERCENT)
            
    elif controller_1.buttonR2.pressing():
            conveyor_feeder.spin(FORWARD,100,PERCENT)
            feeder_motor.spin(FORWARD,100,PERCENT)
            
    else:
        conveyor_feeder.stop()
        feeder_motor.stop()

def arm_control():
    if controller_1.buttonL1.pressing():
            arm.spin(REVERSE)
    elif controller_1.buttonL2.pressing():
            arm.spin(FORWARD)
    else:arm.stop()




    
def dispenser_off():
        dispenser.spin(FORWARD)
        at_max(dispenser,lambda:dispenser.stop)
def dispenser_on():
        dispenser.spin(REVERSE)
        at_max(dispenser,lambda:dispenser.stop)   



def toggle_dispenser():
  
    global dispenser_state

    if dispenser_state:
        dispenser_off()
        print("Dispenser is now OFF (Closed).")
    else:
        dispenser_on()
        print("Dispenser is now ON (Open).")

    # Toggle the state
    dispenser_state = not dispenser_state
    
def detect_and_control_dispenser():
    """
    Continuously checks for the nearest object using middle_vision.
    - Opens the dispenser if a blue object is nearest.
    - Closes the dispenser if a red object is detected.
    """
    while True:
        # Detect nearest blue object
        is_blue_nearest = nearest_obj(BLUE_RING_sig,front_vision, middle_vision)

        # Detect nearest red object
        is_red_detected = nearest_obj(RED_RING_sig, middle_vision)

        if is_red_detected:
            # Close the dispenser if red is detected
            print("Red object detected. Closing dispenser.")
            dispenser_off()
        elif is_blue_nearest:
            # Open the dispenser if blue is nearest
            print("Blue object detected. Opening dispenser.")
            dispenser_on()
        else:
            # If neither is detected, turn the dispenser off for safety
            print("No relevant object detected. Dispenser is off.")
            dispenser_off()

        # Add a short delay to avoid excessive CPU usage
        wait(10, MSEC)










#endregion custom_drive_utils

def pre_autonomous():
    
    arm.set_velocity(100,PERCENT)
    arm.set_stopping(HOLD)
    arm.set_max_torque(100,PERCENT)
    brain.screen.clear_screen(Color.RED)
    
    if brain.battery.capacity() > 70: indicate_ok(brain.battery)
    if front_vision.installed(): indicate_ok(front_vision)
    if back_vision.installed(): indicate_ok(back_vision)




def indicate_ok(part):
    idx = [brain.battery,front_vision,back_vision].index(part)
    
    w = 150
    brain.screen.draw_rectangle(idx * w + idx*10,0,w,700,Color.GREEN)

def is_moving():
    return abs(drivetrain.velocity()) > 0

def when_stucked(callback):
    wait(100,MSEC)
    while not is_done():
       
        if not is_moving():
            print("stucked")
            callback()  
            break
        
        print("moving ",drivetrain.velocity())
        wait(10,MSEC)

def is_done():
    return drivetrain.is_done()


def autonomous():
        
    controller_1.buttonLeft.pressed( lambda: drive_turn(30))
    controller_1.buttonRight.pressed( lambda: drive_turn(30,RIGHT))
    controller_1.buttonUp.pressed( lambda: drive_for(5))
    controller_1.buttonDown.pressed( lambda: drive_for(-5))
    
    controller_1.buttonY.pressed(lambda:look_at(aliance_ring))
    controller_1.buttonB.pressed(lambda:look_at(STAKE_sig,back_vision))
    
    controller_1.buttonR1.pressed(lambda:intake.spin(REVERSE))
    controller_1.buttonR2.pressed(lambda:intake.spin(FORWARD))
    controller_1.buttonL1.pressed(lambda:intake.stop())


    
    controller_1.buttonA.pressed(
        lambda:grabber.close()
    )
    controller_1.buttonX.pressed(
    lambda:grabber.open()
    )
   
    
    wait_time = 800
    max_speed = 17
    

    # look_at(aliance_ring,middle_vision)
    # Setup logic to detect blue_stake_sig and control dispenser
    # intake()
    # detect_and_control_dispenser()

    dispenser_off()
    drive_for(-15,max_speed)
    arm.spin_for(REVERSE,500,wait=False)
    left_turn(40)
    # look_at(stake, back_vision)
    drive_for(-37,15)
    grabber.open()
    wait(wait_time,MSEC)


    drive_for(13,15)
    
    
    # # #11111111111111111111111111
    left_turn(50)
    # # # look_at(aliance_ring,front_vision)
    intake()
    drive_for(30,max_speed)
    wait(1,SECONDS)
    drive_for(-5,max_speed)

    
    
    # # # #2222222222222222222222222222
    left_turn(135)
    drive_for(25,max_speed)
    # # look_at(aliance_ring)
    wait(1,SECONDS)
    drive_for(-5,max_speed)
    
    # # # #######3333333333333333
    right_turn(32)
    # look_at(aliance_ring)
    drive_for(25,max_speed)
    wait(1,SECONDS)
    wait(wait_time,MSEC)
    
    #444444444444444444444

    left_turn(110)
    # look_at(aliance_ring)
    drive_for(22,max_speed)
    drive_for(-12,max_speed)

    # drive_for(23,max_speed,_wait= False)
    # when_stucked(lambda:drivetrain.stop())
    # drive_for(-10,max_speed)
    wait(wait_time,MSEC)
    
    
    # #555555555555555555555
    left_turn(47)
    # look_at(aliance_ring)
    
    drive_for(60,max_speed,_wait=False)
    wait(wait_time,MSEC)
    when_stucked(lambda:drive_for(-15,max_speed))
    
    
    # #66666666666666666666666666
    # right_turn(45)
    # look_at(aliance_ring)
    
    # drive_for(27,max_speed,_wait=False)
    # when_stucked(lambda:drivetrain.stop())
    # drive_for(-10,max_speed)

    # wait(wait_time,MSEC)
    
    # drive_for(15,max_speed,_wait=False)
    # when_stucked(lambda:drivetrain.stop())
    # drive_for(-10,max_speed)
    
    # wait(wait_time,MSEC)
    
    # #deposit
    # left_turn(180)
    # wait(100,MSEC)
    # grabber.close()
    
    
    # # magnaye
    # # drive_for(-20,_wait=False)
    # # when_stucked(lambda:drive_for(15,max_speed))
    

    # # intake(False)
    # # left_turn(15)
    # # drive_for(15,max_speed)

    # # left_turn(25)

    # # drive_for(-40,100,_wait=False)
    # # when_stucked(lambda:drivetrain.stop())
    
    # # # look_at(aliance_ring)
    # # drive_for(50,40)
    # # magnaye


   
    # drive_for(-20,_wait=False)
    # when_stucked(lambda:drive_for(25,max_speed))
    
    # intake(False)
    # right_turn(30)

    # #turn
    # drive_for(-60,100,_wait=False)
    # when_stucked(lambda:drivetrain.stop())
    # drive_for(18,max_speed)
    # right_turn(85)

    # # stake grab
    # drive_for(-50,max_speed)
    # right_turn(45)
    # # look_at(STAKE_sig)
    # drive_for(-48,12)
    # grabber.open()

    # #777777777
    # drive_for(10,max_speed) 
    # right_turn(80)
    # look_at(aliance_ring)
    # drive_for(35, 12)
    # intake(False)
    
    # #flappers
    # flapper_on()
    # right_turn(80)
    # left_turn(80)
    # flapper_off()

    # intake()
    # drive_for(40,max_speed,_wait=False)
    # when_stucked(lambda:drivetrain.stop())
    # intake(False)
    
    # #remove blue
    # #888888888
    # wait(wait_time,MSEC)
    # flapper_on()
    # right_turn(100)
    # flapper_off()
    # intake(True)
    # drive_for(10,max_speed)
    # wait(wait_time,MSEC)

    # # #999999999
    # # right_turn(35)
    # # drive_turn(35, max_speed)

    # # #end
    # # drive_turn(-60, max_speed,_wait=False)
    # # when_stucked(lambda:drivetrain.stop())
    # # intake(False)


    
    # #888888888
    # right_turn(45)
    # right_turn(80)
    # look_at(aliance_ring)
    # drive_for(20, max_speed)
    # wait(wait_time,MSEC)
    # drive_for(-5,max_speed)

    # #dispose
    # right_turn(55)
    # drive_for(-60,max_speed,_wait=False)
    # when_stucked(lambda:drivetrain.stop())
    # grabber.close()

    # #push stake to corner
    # drive_for(-50,40,_wait=False)
    # when_stucked(lambda:drivetrain.stop())
    
    # #end
    # wait(1,SECONDS)
    # drive_for(3,max_speed,_wait=False)
    # intake(False)
    # drive_for(-3,max_speed,_wait=False)
    # drive_for(3,max_speed,_wait=False)
    # # look_at(aliance_ring)
        

# def check_if_aliance(obj_sign:Signature,vision:Vision = middle_vision):
        
#         if is_in_vision(aliance_ring()):
#             wait(1,SECONDS)
        
#         dispenser_on()

     

f_op = False
# def drivers_control():
def user_control():

    # detect_and_control_dispenser()
    arm.set_velocity(100, PERCENT)
    dispenser.set_velocity(100,PERCENT)
    drivetrain.set_stopping(BRAKE)
    
    controller_1.buttonY.pressed(
        lambda:grabber.close()
    )
    controller_1.buttonB.pressed(
       lambda:grabber.open()
    )

    
    # controller_1.buttonDown.pressed(lambda:toggle_dispenser())

    controller_1.buttonA.pressed(lambda:look_at(aliance_ring))
    controller_1.buttonX.pressed(lambda:look_at(STAKE_sig,back_vision))
    
    def extend_flaper():
        global f_op
        f_op = not f_op
        flapper.spin(FORWARD if f_op else REVERSE,10000)
        at_max(flapper,lambda:flapper.stop)
        
    def toggle_dispense():
        global f_op
        f_op = not f_op
        dispenser.spin(FORWARD if f_op else REVERSE,10000)
        at_max(dispenser,lambda:dispenser.stop)

    controller_1.buttonDown.pressed(toggle_dispense)
    controller_1.buttonRight.pressed(extend_flaper)
    
    while True:
        conveyor_control()
        arm_control()
        drivetrain_movement(from_controller=True,speed_mul=0.2 if controller_1.buttonDown.pressing() else 1)

    # controller_1.buttonY.pressed(
    #     lambda:grabber.close()
    # )
    # controller_1.buttonB.pressed(
    #    lambda:grabber.open()
    # )

    # controller_1.buttonA.pressed(lambda:look_at(aliance_ring))
    # controller_1.buttonX.pressed(lambda:look_at(STAKE_sig,back_vision))
    
    
    

    # while True:
    #     conveyor_control()
    #     arm_control()
        
        
    #     drivetrain_movement(from_controller=True,speed_mul=0.2 if controller_1.buttonDown.pressing() else 1)




comp = Competition(user_control,autonomous)
pre_autonomous()

#region penas



    # while True:
    #     conveyor_control()
    #     arm_control()
        
        
    #     drivetrain_movement(from_controller=True,speed_mul=0.2 if controller_1.buttonDown.pressing() else 1)


# create competition instance
comp = Competition(user_control, autonomous)

pre_autonomous()




#region penas


