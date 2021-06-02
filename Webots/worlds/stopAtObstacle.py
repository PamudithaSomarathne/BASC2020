from controller import Robot
from controller import InertialUnit
from controller import LED
from controller import DistanceSensor

num_ramps=0
inc=0
dec=0
if __name__ == "__main__":
    robot = Robot()
    imu=InertialUnit('inertial unit')
    led_incline=robot.getDevice('led_green')
    led_decline=robot.getDevice('led_red')
    s1 = robot.getDevice("so");
    
    timestep = 64
    max_speed = 6.28
    scale = 0.6
       
    back_left_motor= robot.getDevice('back left wheel')
    back_right_motor= robot.getDevice('back right wheel')
    front_left_motor= robot.getDevice('front left wheel')
    front_right_motor= robot.getDevice('front right wheel')
    back_left_motor.setPosition(float('inf'))
    back_left_motor.setVelocity(0.0)
    back_right_motor.setPosition(float('inf'))
    back_right_motor.setVelocity(0.0)
    front_left_motor.setPosition(float('inf'))
    front_left_motor.setVelocity(0.0)
    front_right_motor.setPosition(float('inf'))
    front_right_motor.setVelocity(0.0)
    
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        back_left_motor.setVelocity(scale*max_speed)
        back_right_motor.setVelocity(scale*max_speed)
        front_left_motor.setVelocity(scale*max_speed)
        front_right_motor.setVelocity(scale* max_speed)
        imu.enable(20)
        s1.enable(20)
        value1=imu.getRollPitchYaw()
        
        
        if value1[0]>0.5 :
            if led_incline.get()==0:
                inc=inc+1
            led_incline.set(1)
        elif value1[0]<-0.5:
            if led_decline.get()==0:
                dec=dec+1
                if dec==1 and inc==0:
                    dec=dec-1                      
            led_decline.set(1)
            
            if inc==1 & dec==1:
                num_ramps=num_ramps+1
                inc=0
                dec=0
        else: 
            led_incline.set(0)
            led_decline.set(0)
        
        
            
        if s1.getValue()>900.0:
            back_left_motor.setVelocity(0)
            back_right_motor.setVelocity(0)
            front_left_motor.setVelocity(0)
            front_right_motor.setVelocity(0)
            imu.disable()
            s1.disable()
            print('Number of ramps : ', num_ramps)
            break
    # Enter here exit cleanup code.
