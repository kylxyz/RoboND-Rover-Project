import numpy as np
import time
import copy

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Offset to hug the left wall
    offset = 0
    if Rover.home is None:
        Rover.home = copy.copy(Rover.pos)
    # Verify that the rover actually at least 2 meters from the home position to increase 
    # the chances of finding a wall
    if not Rover.left_home and Rover.home is not None:
        if distance_to(Rover.pos, Rover.home) > 10:
            print('I\'m hugging the left wall')
            Rover.left_home = True
            # Activate the wall hugging function
            Rover.hug_wall = True

    # Set the offset to hug the left wall
    if Rover.hug_wall:
        # Increase the offset multiplier gradually in order to avoid spinning if starting on a big area
        if Rover.offset_multiplier < Rover.max_offset_multiplier:
            Rover.offset_multiplier += 0.1
        offset = Rover.offset_multiplier * np.std(Rover.nav_angles)
    else:
        offset = 0

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        if Rover.samples_collected == 6 and distance_to(Rover.home, Rover.pos) < 15:
            Rover.going_home = True
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            stuck = False
            if Rover.last_pos is None:
                Rover.last_pos = copy.copy(Rover.pos)
                Rover.last_pos_tm = time.time()
            check = time.time() - Rover.last_pos_tm
            if check >= 15.0:
                if Rover.vel > -0.1 and Rover.vel < 0.1 and not Rover.picking_up:
                    stuck = True
                else:
                    Rover.unstuck_method = 0
                    Rover.tm_stuck = None
                    Rover.tm_stuck_sec = 0
                    if Rover.left_home:
                        Rover.hug_wall =  True
                Rover.last_pos = copy.copy(Rover.pos)
                Rover.last_pos_tm = time.time() 
            if Rover.left_home and Rover.steer == 15.0:
                if Rover.tm_spin is None:
                    Rover.tm_spin = time.time()
                Rover.tm_spin_sec += time.time() - Rover.tm_spin
                if Rover.tm_spin_sec > 800.0 and Rover.unstuck_method == 0:
                    print('spining for: '+str(Rover.tm_spin_sec)+' sec')
                    Rover.hug_wall = False
                    Rover.offset_multiplier = 0.0
                    Rover.tm_spin = None
                    Rover.tm_spin_sec = 0
                    Rover.tm_unspin = time.time()
            else:
                if Rover.tm_unspin is None:
                    Rover.tm_spin = None
                    Rover.tm_spin_sec = 0
                    if Rover.left_home:
                        Rover.hug_wall = True
                elif Rover.tm_unspin_sec > 200.0:
                    print('I\'m turning on the hug function again...') 
                    Rover.tm_unspin = None
                    Rover.tm_unspin_sec = 0
                else:
                    Rover.tm_unspin_sec += time.time() - Rover.tm_unspin
            # Check if we are near gold
            if Rover.near_sample:
                Rover.previous_mode = copy.copy(Rover.mode)
                Rover.mode = 'gold'
            elif stuck:
                # Set mode to "stuck" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.previous_mode = copy.copy(Rover.mode)
                Rover.mode = 'stuck'
            # Check if we have any gold on sight and only on the left side
            elif Rover.gold_angles is not None and np.min(Rover.gold_dists) < 50.0:
                Rover.stop_tm = None
                Rover.throttle = 0.1
                # Set steering to average angle without offset to increase the chances of picking up the gold
                # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                if np.min(Rover.gold_dists) < 40.0:
                    Rover.pre_gold_pos = copy.copy(Rover.pos)
                    Rover.pre_gold_yaw = copy.copy(Rover.yaw)
                    # Hit the brakes to give the rover a chance to correct angle before trying to pick up the gold
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    # Rover.steer = 0
                    Rover.previous_mode = copy.copy(Rover.mode)
                    Rover.mode = 'gold'
            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                if Rover.going_home:
                    # Set steering to average angle clipped to the range +/- 15 without offset
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                else:
                    mean = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    # if mean < -12.0:
                    #     offset = offset / 2
                    # Set steering to average angle clipped to the range +/- 15 with offset
                    Rover.steer = np.clip(np.mean((Rover.nav_angles + offset) * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
            if Rover.going_home and distance_to(Rover.home, Rover.pos) < 5:
                print('very close from home')
                Rover.mode = 'home'

        # If we're stuck mode let's try to get out of it
        elif Rover.mode == 'stuck':
            if Rover.tm_stuck is None:
                Rover.tm_stuck = time.time()
            Rover.tm_stuck_sec += time.time() - Rover.tm_stuck
            # If we have been in stuck state for less than 10 seconds try reversing
            if Rover.unstuck_method == 0:
                print('I have been stuck for '+str(Rover.tm_stuck_sec)+'sec, reversing')
                # Release the brake to allow reversing
                Rover.brake = 0
                Rover.steer = 0
                # Try reversing to get unstuck
                Rover.throttle = -(Rover.throttle_set/2)
                if Rover.tm_stuck_sec > 150:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0.1
                    Rover.unstuck_method = 1
                    Rover.mode = 'stop'
                    Rover.tm_stuck = None
                    Rover.tm_stuck_sec = 0
                    Rover.hug_wall = False
            # If are still stuck after 10 seconds but less than 30 seconds try spinning to the right
            elif Rover.unstuck_method == 1:
                steer = -15
                if len(Rover.nav_angles) >= Rover.go_forward:
                    if np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) > 0:
                        steer = 15
                print('I\'m still stuck after '+str(Rover.tm_stuck_sec)+'sec, spinning: '+str(steer)+'degrees')
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = steer
                # If we're stucked but see sufficient navigable terrain in front or spend too long then go!
                if len(Rover.nav_angles) >= Rover.go_forward or Rover.tm_stuck_sec > 100:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.unstuck_method = 2
                    Rover.mode = 'stop'
                    Rover.tm_stuck = None
                    Rover.tm_stuck_sec = 0
                    Rover.hug_wall = False
            elif Rover.unstuck_method == 2:
                print('I\'m still stuck after '+str(Rover.tm_stuck_sec)+'sec, spinning right')
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = -15
                # If we're stucked but see sufficient navigable terrain in front or spend too long then go!
                if Rover.tm_stuck_sec > 20:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.unstuck_method = 3
                    Rover.mode = 'stop'
                    Rover.tm_stuck = None
                    Rover.tm_stuck_sec = 0
                    Rover.hug_wall = False
            elif Rover.unstuck_method == 3:
                print('I will ram full speed backwards')
                Rover.brake = 0
                Rover.steer = 0
                Rover.throttle = -2.0
                if Rover.tm_stuck_sec > 80:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.unstuck_method = 4
                    Rover.mode = 'stop'
                    Rover.tm_stuck = None
                    Rover.tm_stuck_sec = 0
                    Rover.hug_wall = False
            elif Rover.unstuck_method == 4:
                print('I will ram full speed ahead')
                Rover.brake = 0
                Rover.steer = 0
                Rover.throttle = 2.0
                if Rover.tm_stuck_sec > 100:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.unstuck_method = 5
                    Rover.mode = 'stop'
                    Rover.tm_stuck = None
                    Rover.tm_stuck_sec = 0
                    Rover.hug_wall = False
            else:
                print('I will stop and try again')
                # Reset the previous mode to None
                Rover.previous_mode = None
                Rover.tm_stuck = None
                Rover.tm_stuck_sec = 0
                Rover.unstuck_method = 0
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
                Rover.hug_wall = False

        # If we struck gold let's pick it
        elif Rover.mode == 'gold':
            if Rover.pre_gold_pos is None:
                Rover.pre_gold_pos = copy.copy(Rover.pos)
            if Rover.tm_gold is None:
                Rover.tm_gold = time.time()
            check = time.time() - Rover.tm_gold
            if check > 80:
                Rover.mode = 'stuck'
                Rover.unstuck_method = 2
                Rover.tm_stuck = None
                Rover.tm_stuck_sec = 0
            # print('we have been on gold for: '+str(check)+' sec')
            # Stop if we are near the sample so we can pick it
            if Rover.near_sample:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
            # Verify that we are still looking at the gold
            elif Rover.gold_angles is not None and np.min(Rover.gold_dists) < 50.0:
                if Rover.pre_gold_cnt is None:
                    Rover.pre_gold_cnt = copy.copy(Rover.samples_collected)
                Rover.pre_gold_angles.append(np.clip(np.mean(Rover.gold_angles * 180/np.pi), -15, 15))
                Rover.steer = np.clip(np.mean(Rover.gold_angles * 180/np.pi), -15, 15)
                print('navigation: '+str(np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)))
                print('gold: '+str(np.clip(np.mean(Rover.gold_angles * 180/np.pi), -15, 15)))
                # print('Gold distance is '+str(np.min(Rover.gold_dists)))
                # print('Is close to gold: '+str(Rover.near_sample))
                # reduce the throttle as we get close to the sample
                Rover.throttle = 0.15
                Rover.brake = 0
            elif len(Rover.pre_gold_angles) > 0 or (distance_to(Rover.pre_gold_pos, Rover.pos) > 2 and distance_to(Rover.pre_gold_pos, Rover.pos) < 3):
                print('We have gold angles amount of: '+str(len(Rover.pre_gold_angles)))
                print('distance to pre gold pos: '+str(distance_to(Rover.pre_gold_pos, Rover.pos)))
                steer = 0
                if len(Rover.pre_gold_angles) > 0:
                    steer = Rover.pre_gold_angles.pop()
                Rover.brake = 0
                Rover.steer = steer
                Rover.throttle = -0.2
                if distance_to(Rover.pre_gold_pos, Rover.pos) < 2 or distance_to(Rover.pre_gold_pos, Rover.pos) > 3:
                    Rover.brake = Rover.brake_set
                    Rover.throttle = 0
                    Rover.steer = 0
                    Rover.pre_gold_angles = []
            elif Rover.gold_angles is None:
                if Rover.pre_gold_cnt == Rover.samples_collected:
                    steer = 0
                    Rover.brake = 0
                    Rover.steer = steer
                    Rover.throttle = -0.1
                    if check > 10:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.mode = 'stuck'
                        Rover.tm_gold = None
                        Rover.pre_gold_cnt = None
                        Rover.pre_gold_pos = None
                        Rover.unstuck_method = 0
                        Rover.tm_stuck = None
                        Rover.tm_stuck_sec = 0
                    print('We lost gold, retrying ')
                else:
                    print('There are no more angles')
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.mode = 'forward'
                    Rover.tm_gold = None
                    Rover.pre_gold_cnt = None
                    Rover.pre_gold_pos = None
                    Rover.unstuck_method = 0
                    Rover.tm_stuck = None
                    Rover.tm_stuck_sec = 0
   
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            if Rover.stop_tm is None:
                Rover.stop_tm = time.time()
            check = time.time() - Rover.stop_tm
            if check >= 15.0:
                Rover.throttle = 0
                Rover.mode = 'stuck'
                Rover.unstuck_method = 0 
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
                    Rover.stop_tm = None
        elif Rover.mode == 'home':
            # If we're in home mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

# Find the distance between 2 positions
def distance_to(pose_a, pose_b):
    dx = pose_a[0] - pose_b[0]
    dy = pose_a[1] - pose_b[1]
    
    return np.sqrt(dx**2 + dy**2)