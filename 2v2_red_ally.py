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

conveyor_feeder_motor_a = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
conveyor_feeder_motor_b = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)
feeder_motor = Motor(Ports.PORT1,GearSetting.RATIO_18_1,False)
conveyor_feeder = MotorGroup(conveyor_feeder_motor_a)
arm_lock = Motor(Ports.PORT15,GearSetting.RATIO_18_1,False)


left_motor_arm = Motor(Ports.PORT20,GearSetting.RATIO_18_1,False)
right_motor_arm = Motor(Ports.PORT16,GearSetting.RATIO_18_1,True)
arm = MotorGroup(left_motor_arm,right_motor_arm)

BLUE_RING_sig = Signature(1, -4393, -2133, -3263,-1, 4567, 2283,1.1, 0)
RED_RING_sig = Signature(2, 6539, 10627, 8583,-2007, -1013, -1510,2.5, 0)
STAKE_sig = Signature(3, -2299, -201, -1250,-7495, -5343, -6419,2.5, 0)
front_vision = Vision(Ports.PORT6, 50, BLUE_RING_sig, RED_RING_sig, STAKE_sig)
back_vision = Vision(Ports.PORT3, 50, BLUE_RING_sig, RED_RING_sig, STAKE_sig)

grabber = Pneumatics(brain.three_wire_port.a)
aliance_ring = RED_RING_sig
flapper = Motor(Ports.PORT10,GearSetting.RATIO_18_1,False)

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






def switch_team_color():


    global aliance_ring


    if aliance_ring == BLUE_RING_sig:
       
        brain.screen.clear_screen(Color.RED)
        aliance_ring = RED_RING_sig
        return
   
    brain.screen.clear_screen(Color.BLUE)
    aliance_ring = BLUE_RING_sig




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
        if (abs(vel) < 0.5 and counter > 5) : break

        # drivetrain_movement(vel)
        counter+=1
        wait(10,MSEC)
    
    looking = False
    drivetrain.stop()
     


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
   
def left_turn(deg,vel=100):
    drive_turn(deg,LEFT,vel)  

def right_turn(deg,vel=100):
    drive_turn(deg,RIGHT,vel)    

def drive_for(dist:float,velocity:float = 10,_wait=True):
    adjustment = 0.25
    drivetrain.drive_for(FORWARD,dist * adjustment,INCHES,velocity,PERCENT,_wait)
    if not _wait : wait(70,MSEC)



def drivetrain_movement(side_speed=0,forward_speed=0,from_controller:bool = False,speed_mul = 1):

    global looking
    
    if from_controller:
        
        if abs(controller_1.axis1.position() + controller_1.axis3.position()) <= 0 and looking :
            

            return
        
        side_speed = controller_1.axis1.position()
        forward_speed = controller_1.axis3.position()  


    # target_right_drive_vel = (forward_speed - side_speed) * speed_mul
    # target_left_drive_vel = (forward_speed + side_speed)* speed_mul
    
    # right_drive_smart.spin(FORWARD,target_right_drive_vel,PERCENT)
    # left_drive_smart.spin(FORWARD,target_left_drive_vel,PERCENT)


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
            feeder_motor.spin(FORWARD,100,PERCENT)
            
    elif controller_1.buttonR2.pressing():
            conveyor_feeder.spin(FORWARD,100,PERCENT)
            feeder_motor.spin(REVERSE,100,PERCENT)
            
    else:
        conveyor_feeder.stop()
        feeder_motor.stop()

def arm_control():
    if controller_1.buttonL1.pressing():
            arm.spin(REVERSE)
    elif controller_1.buttonL2.pressing():
            arm.spin(FORWARD)
    else:arm.stop()

f_op =False
def extend_flaper():
        global f_op
        f_op = not f_op
        flapper.spin(FORWARD if f_op else REVERSE,10000)
        at_max(flapper,lambda:flapper.stop)
#endregion custom_drive_utils



def pre_autonomous():
    
    
    arm.set_velocity(100,PERCENT)
    arm.set_stopping(HOLD)
    arm.set_max_torque(100,PERCENT)
    # brain.screen.clear_screen(Color.RED)
    
    if brain.battery.capacity() > 70: indicate_ok(brain.battery)
    if front_vision.installed(): indicate_ok(front_vision)
    if back_vision.installed(): indicate_ok(back_vision)
    brain.screen.clear_screen(Color.RED)

   
    def st():
        if not comp.is_enabled():
            switch_team_color()
            print("switch")
    
    def display_stats():

        if comp.is_enabled():return

       
        indicate_ok(brain.battery)
        if front_vision.installed(): 
            indicate_ok(front_vision)
        if back_vision.installed(): 
            indicate_ok(back_vision)
    
    
    display_stats()
    brain.screen.pressed(st)
    # wait(1,SECONDS)
    
    
# def indicate_ok(part):
#     idx = [brain.battery,front_vision,back_vision].index(part)
    
#     w = 150
#     brain.screen.draw_rectangle(idx * w + idx*10,0,w,700,Color.GREEN)

def indicate_ok(part):
    idx = [brain.battery,front_vision,back_vision].index(part)
    
    w = 150
    h= 220
    
    brain.screen.draw_rectangle(idx * w + idx*10,0,w,h,Color.GREEN)

    if idx ==0:
        if brain.battery.capacity() < 70: 
            brain.screen.draw_rectangle(idx * w + idx*10,0,w,h,Color.RED)
        batt = brain.battery.capacity()
        brain.screen.draw_rectangle(idx * w + idx*10,0,w,h * (100 - batt )/100 ,Color.BLACK)

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

def at_max(motor:Motor,callback):

    wait(200,MSEC)

    while not motor.is_spinning():
        print(abs(motor.velocity()),"  ",motor.is_spinning())
        if not abs(motor.velocity()) > 0:
            print("maxed")
            callback()  
            break
        wait(10,MSEC)

def is_done():
    return drivetrain.is_done()


def autonomous():
   
    wait_time= 800
    max_spd = 16



    def no_stake():

        grabber.close()

        # left_turn(40)
        # look_at(STAKE_sig)
        # extend_flaper()
        drive_for(23,100)
        # look_at(STAKE_sig)
        right_turn(90)
        # right_turn(90)
        # right_turn(180,15)
        look_at(STAKE_sig,back_vision)
        drive_for(-30,max_spd)
        # extend_flaper()
        grabber.open()
        # extend_flaper()
        conveyor_feed()
        wait(300, MSEC)


        left_turn(8, 15)
        # look_at(aliance_ring)
        # right_turn(10)
        intake()
        drive_for(36,100)
        
        # look_at(aliance_ring)
        # drive_for(5, max_spd)

    def got_stake():
        intake()
        wait(100,MSEC)
        right_turn(30)
        look_at(opposing_ring())
        drive_for(25)
   
    # drive_for(-35,100,_wait=False)
    # arm.spin_for(REVERSE,860,wait=False)
    
    # while drivetrain.is_moving():wait(10,MSEC)

    # left_turn(35)
    # look_at(STAKE_sig,back_vision)
    # left_turn(5)
    # drive_for(-13,10)
    # grabber.open()

    # drive_for(15,20)
    # right_turn(20)

    # stake = nearest_obj(STAKE_sig,back_vision)
    
    # if  stake and stake.width > 170:
    #     return got_stake()
    
    # no_stake()
    arm.spin_for(REVERSE,860,wait=False)
    drive_for(-10)
    right_turn(90)
    drive_for(-20)
    grabber.open()
    intake()
    drive_for(38, 100)

    wait(1500, MSEC)
    feeder_motor.stop()

    return
    wait_time= 800
    max_spd = 16

    intake()
    arm.spin_for(REVERSE,900,wait=False)
    drive_for(-30)
    grabber.open()
    left_turn(52)
    drive_for(12)
    wait(1000, MSEC)
    feeder_motor.stop()
    drive_for(-10)
    left_turn(10)
    feeder_motor.spin(FORWARD)
    wait(500,MSEC)
    
    look_at(aliance_ring)
    
    drive_for(10)
    wait(1000, MSEC)
    arm.spin_for(FORWARD,450,wait=True)
    drive_for(-15)
    right_turn(70)
    drive_for(30)
    
    wait(500,MSEC)
    look_at(aliance_ring)
    feeder(True)
    drive_for(25, 50)
    wait(500, MSEC)
    feeder_motor.spin(FORWARD)
    wait(1000, MSEC)
    drive_for(-15)
    right_turn(75)
    feeder_motor.spin(FORWARD)
    drive_for(20)
    left_turn(60)
    drive_for(20, 50)
    return
    drivetrain.drive_for(REVERSE, 6.25, DistanceUnits.IN, 90, PERCENT)
    arm.spin_for(REVERSE,500,wait=False)
    wait(200,MSEC)
    left_turn(20)
    wait(200,MSEC)
    drive_for(-30)
    grabber.open()
    wait(200,MSEC)
    drive_for(15)
    wait(200,MSEC)
    intake()
    wait(200,MSEC)
    right_turn(25)
    drive_for(30, 30)
    wait(200,MSEC)
    left_turn(80)
    drive_for(50, 30)
    
def lock_arm():
    arm_lock.spin_for(FORWARD,20,DEGREES,wait=False)
    lambda:arm_lock.stop(HOLD)

def release_arm():
    arm_lock.spin_for(REVERSE,20,DEGREES,wait=False)
    lambda:arm_lock.stop(HOLD)

    
   
   

def drivers_control():
    arm.set_velocity(100, PERCENT)
    arm_lock.set_velocity(100,PERCENT)
    drivetrain.set_stopping(BRAKE)
    
    controller_1.buttonY.pressed(
        lambda:grabber.close()
    )
    controller_1.buttonB.pressed(
       lambda:grabber.open()
    )

    
    
    controller_1.buttonA.pressed(release_arm)
    controller_1.buttonX.pressed(lock_arm)
    
   
        
    
    controller_1.buttonRight.pressed(extend_flaper)
    
    while True:
        conveyor_control()
        arm_control()
        # drivetrain_movement(from_controller=True,speed_mul=0.2 if controller_1.buttonDown.pressing() else 1)

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




comp = Competition(drivers_control,autonomous)
# comp = Competition(autonomous, drivers_control)
pre_autonomous()

#region penas


