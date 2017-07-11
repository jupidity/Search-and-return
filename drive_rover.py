import argparse
import shutil
import base64
from datetime import datetime
import os
import cv2
import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from PIL import Image
from flask import Flask
from io import BytesIO, StringIO
import json
import pickle
import matplotlib.image as mpimg
import time

# Import functions for perception and decision making
from perception import perception_step
from decision import decision_step
from supporting_functions import update_rover, create_output_images
# Initialize socketio server and Flask application 
# (learn more at: https://python-socketio.readthedocs.io/en/latest/)
sio = socketio.Server()
app = Flask(__name__)

# Read in ground truth map and create 3-channel green version for overplotting
# NOTE: images are read in by default with the origin (0, 0) in the upper left
# and y-axis increasing downward.
ground_truth = mpimg.imread('../calibration_images/map_bw.png')
# This next line creates arrays of zeros in the red and blue channels
# and puts the map into the green channel.  This is why the underlying 
# map output looks green in the display image
ground_truth_3d = np.dstack((ground_truth*0, ground_truth*255, ground_truth*0)).astype(np.float)
# Define RoverState() class to retain rover state parameters
class RoverState():
    
    # function that places the rover at a specific position and orientation
    # as long as there are no obstacles present
    def move_to_pos(self, des_pos, des_orientation):
        
        # make sure the rover can complete the journey before moving
        if (self.traj is None):
            self.steer = 15 # if not, turn until a trajectory is calcutaed by the perceptiopn step
            
        # next, check that the rover is not in the desired x,y position    
        elif ((abs(self.pos[0] - des_pos[0] ) < self.loc_tolerance) \
               and (abs(self.pos[1] - des_pos[1] ) < self.loc_tolerance) ):
            
            # if not, continue to follow the trajectory
            Rover.des_pos = self.traj.pop()
        
        # if the rover is in the desired location, make sure the orientaiton is correct
        elif (abs(self.orientation - des_orientation) > self.angle_tolerance):
            self.set_orientation(des_orientation)
        
        # if the orientation is correct and the robot is in the desired location, return position_set
        if(self.mode == 'orientation_set'):
            self.mode = 'position_set'
        
        
        
    def set_orientation(self, desired_orientation):
            
            # make sure the rover is stopped
            self.throttle = 0

            current_yaw = self.yaw

            if(abs(self.yaw - desired_orientation) > self.angle_tolerance):
                self.steer = 15
            else: 
                self.mode = 'orientation_set'
                
                
    def __init__(self):
        self.start_time = None # To record the start time of navigation
        self.total_time = None # To record total duration of naviagation
        self.img = None # Current camera image
        self.pos = None # Current position (x, y)
        self.yaw = None # Current yaw angle
        self.pitch = None # Current pitch angle
        self.roll = None # Current roll angle
        self.vel = None # Current velocity
        self.steer = 0 # Current steering angle
        self.throttle = 0 # Current throttle value
        self.brake = 0 # Current brake value
        #self.nav_angles = None # Angles of navigable terrain pixels
        #self.nav_dists = None # Distances of navigable terrain pixels
        #self.sample_angles = None # Angles towards observed samples
        #self.sample_dists = None # distances towards observed samples
        self.sample_stop_forward = 3 # distance from samples that the rover should start applying brakes
        self.angle_tolerance = 9 # acceptable error for desired steering angle 
        self.ground_truth = ground_truth_3d # Ground truth worldmap
        self.mode = 'initialize' # Current mode (can be forward or stop)
        self.throttle_set = 0.05 # Throttle setting when accelerating
        self.brake_set = 5 # Brake setting when braking
        #self.task = None # differentiating between nav and samples
        self.steering_set = 1 # steering angle set when reorienting [-15,15]
        self.start_yaw = 0 # original starting orientation of the rover
        self.end_pos = None # [x,y] position that the rover must end in
        self.first_orientation = 300# 40 228 # starting orientation for the rover
        self.angle_tolerance = 5 # angle in degrees the starting yaw can be wrong by
        # The stop_forward and go_forward fields below represent total count
        # of navigable terrain pixels.  This is a very crude form of knowing
        # when you can keep going and when you should stop.  Feel free to
        # get creative in adding new fields or modifying these!
        self.stop_forward = 250 # Threshold to initiate stopping
        self.go_forward = 1500 # Threshold to go forward again
        self.max_vel = 2 # Maximum velocity (meters/second)
        self.error = 0 # error in Rover y pos
        self.tau_d = 0 # derivative error control parameter (this is super high since its accounting for 1/dt)
        self.tau_p = .01# proportional error control parameter
        self.des_y_pos = 0 # desired y position for the rover
        self.des_x_pos = None # desired x position for the rover, used only when specifying a location to move towards
        self.x_range = 50 # rangin in x to determine desired path for Rover
        self.avg_y_pos = [0] * 10 # window of median y pos to pass as a desired state
        self.go_threshold = .4 # yaw angle range from 0 that the desired path must be in before getting out of stop mode
        self.offset_factor = .45 # offset for rover to make it a wall crawler
        self.pos_before_rock = None # [x,y] position of Rover before starting a rock grab
        self.orintation_before_rock = 0 # orientation in degrees of Rover before starting a rock grab
        #self.path_x = None
        #self.path_y = None
        self.navigateable_path_pixels = 0 # number of pixels in the navigatable path
        self.offset = 0 # offset from median in attempt to favor right turns
        # Image output from perception step
        # Update this image to display your intermediate analysis steps
        # on screen in autonomous mode
        self.vision_image = np.zeros((160, 320, 3), dtype=np.float) 
        # Worldmap
        # Update this image with the positions of navigable terrain
        # obstacles and rock samples
        self.worldmap = np.zeros((200, 200, 3), dtype=np.float) 
        self.samples_pos = None # To store the actual sample positions
        self.samples_found = 0 # To count the number of samples found
        self.near_sample = 0 # Will be set to telemetry value data["near_sample"]
        self.picking_up = 0 # Will be set to telemetry value data["picking_up"]
        self.send_pickup = False # Set to True to trigger rock pickup
# Initialize our rover 
Rover = RoverState()

# Variables to track frames per second (FPS)
# Intitialize frame counter
frame_counter = 0
# Initalize second counter
second_counter = time.time()
fps = None



# Define telemetry function for what to do with incoming data
@sio.on('telemetry')
def telemetry(sid, data):

    global frame_counter, second_counter, fps
    frame_counter+=1
    # Do a rough calculation of frames per second (FPS)
    if (time.time() - second_counter) > 1:
        fps = frame_counter
        frame_counter = 0
        second_counter = time.time()
    print("Current FPS: {}".format(fps))

    if data:
        global Rover
        # Initialize / update Rover with current telemetry
        Rover, image = update_rover(Rover, data)

        if np.isfinite(Rover.vel):

            # Execute the perception and decision steps to update the Rover's state
            if(Rover.picking_up == 0):
                Rover = perception_step(Rover)
                Rover = decision_step(Rover)


            # Create output images to send to server
            out_image_string1, out_image_string2 = create_output_images(Rover)

            # The action step!  Send commands to the rover!
            commands = (Rover.throttle, Rover.brake, Rover.steer)
            send_control(commands, out_image_string1, out_image_string2)
 
            # If in a state where want to pickup a rock send pickup command
            if Rover.send_pickup and not Rover.picking_up:
                send_pickup()
                # Reset Rover flags
                Rover.send_pickup = False
        # In case of invalid telemetry, send null commands
        else:

            # Send zeros for throttle, brake and steer and empty images
            send_control((0, 0, 0), '', '')

        # If you want to save camera images from autonomous driving specify a path
        # Example: $ python drive_rover.py image_folder_path
        # Conditional to save image frame if folder was specified
        if args.image_folder != '':
            timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
            image_filename = os.path.join(args.image_folder, timestamp)
            image.save('{}.jpg'.format(image_filename))

    else:
        sio.emit('manual', data={}, skip_sid=True)

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control((0, 0, 0), '', '')
    sample_data = {}
    sio.emit(
        "get_samples",
        sample_data,
        skip_sid=True)

def send_control(commands, image_string1, image_string2):
    # Define commands to be sent to the rover
    data={
        'throttle': commands[0].__str__(),
        'brake': commands[1].__str__(),
        'steering_angle': commands[2].__str__(),
        'inset_image1': image_string1,
        'inset_image2': image_string2,
        }
    # Send commands via socketIO server
    sio.emit(
        "data",
        data,
        skip_sid=True)
    eventlet.sleep(0)
# Define a function to send the "pickup" command 
def send_pickup():
    print("Picking up")
    pickup = {}
    sio.emit(
        "pickup",
        pickup,
        skip_sid=True)
    eventlet.sleep(0)
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument(
        'image_folder',
        type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the images from the run will be saved.'
    )
    args = parser.parse_args()
    
    #os.system('rm -rf IMG_stream/*')
    if args.image_folder != '':
        print("Creating image folder at {}".format(args.image_folder))
        if not os.path.exists(args.image_folder):
            os.makedirs(args.image_folder)
        else:
            shutil.rmtree(args.image_folder)
            os.makedirs(args.image_folder)
        print("Recording this run ...")
    else:
        print("NOT recording this run ...")
    
    # wrap Flask application with socketio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
eventlet.wsgi.server(eventlet.listen(('', 4567)), app)  
    
    

