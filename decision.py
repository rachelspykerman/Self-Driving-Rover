import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            Rover.flag = 0
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
                    
        # Check Rover mode if in pickup mode
        elif Rover.mode == 'pickup':
            print("-----------IN PICK UP MODE-----------\n\n\n")
            # if in pickup mode, update steering
            Rover.steer = np.mean(Rover.nav_angles * 180/np.pi)
            # flag is used to determine when the Rover has completed picking up a rock
            # if Rover is near the sample and is stopped and is not already picking up a rock, then let's pick up a rock
            if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up and Rover.flag == 0:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.steer = 0
                # pickup rock
                Rover.send_pickup = True
                # make Rover mode stop so we exit pickup mode, and the Rover will go to stop logic section
                Rover.mode = 'stop'
                Rover.flag = 1
            # if we are near the sample but Rover is not stopped (vel != 0) then we need to stop the Rover 
            elif Rover.near_sample and Rover.flag == 0:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.steer = 0
            # if Rover is near the sample and we are finished picking up a rock, make mode stop
            elif Rover.near_sample and Rover.flag == 1:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.steer = 0
                Rover.mode = 'stop'
                
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            Rover.flag = 0
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        #Rover.send_pickup = True
        Rover.samples_found += 1
    
    return Rover

