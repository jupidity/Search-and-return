import numpy as np
                
    

    
    
def decision_step(Rover):
    
    
    
    # check if the robot is in initialize mode
    if (Rover.mode == 'initialize'):
        #save the current values to return to
        Rover.end_pos = Rover.pos
        Rover.start_yaw= Rover.yaw
        # set mode to start
        Rover.mode = 'start'
    
    # check if te rover is in start mode
    if (Rover.mode == 'start'):
        
        Rover.set_orientation(Rover.first_orientation)
        
        if Rover.mode == 'orientation_set': # then the orientation has been successfully set
            Rover.mode = 'forward'
               
                    
    #elif ((Rover.nav_angles is not None) and ( Rover.sample_angles is None)):
    if (Rover.navigateable_path_pixels > 0):    
        # Check for Rover.mode status
        #Rover.task = 'nav'
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if Rover.navigateable_path_pixels >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering as proportional control centered around a point offset from the mean
                # by a degree relative to the width of the traverseable path. In order to limit shaking, there should be 
                # some derivative control as well. If we save the last steering mean as a class var, we can determine 
                # a momentary derivative by approximation. We just need to have an estimate for the time elasped
                # at each step
                # Set steering 
                
               
                last_error = Rover.error # save the error for the D state calc
                
                Rover.error = Rover.offset-Rover.des_y_pos
                
                steer = - Rover.tau_p * Rover.error - Rover.tau_d * (Rover.error-last_error)
                #print('error: ' + str(Rover.error)  +', desired: ' + str(Rover.des_pos) + ', steer: ' + str(steer) + \
                #         ', d_error: ' + str(Rover.error-last_error))
                Rover.steer = np.clip( (steer * 180/np.pi) , -15, 15)
                #Rover.steer += np.clip( (steer * 180/np.pi) , -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif (Rover.navigateable_path_pixels < Rover.stop_forward):
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif (Rover.mode == 'stop'):
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if (Rover.navigateable_path_pixels < Rover.go_forward):
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if Rover.navigateable_path_pixels >= Rover.go_forward and (Rover.des_y_pos < Rover.go_threshold) :
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steering 
                    
                    last_error = Rover.error # save the error for the D state calc
                    
                    Rover.error = Rover.offset-Rover.des_y_pos
                    steer = - Rover.tau_p * Rover.error - Rover.tau_d * (Rover.error-last_error)
                    #print('error: ' + str(Rover.error)  +', desired: ' + str(Rover.des_pos) + ', steer: ' + str(steer) + \
                    #      ', d_error: ' + str(Rover.error-last_error))
                    Rover.steer = np.clip( (steer * 180/np.pi) , -15, 15)
                    
                    Rover.mode = 'forward'                
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    
    
    return Rover