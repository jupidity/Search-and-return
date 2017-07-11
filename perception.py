import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160,160,160)):
    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    

    # define range of path color in HSV
    lower_path = np.array([0,0,180])
    upper_path = np.array([111,48,255])
    
    # define range of path color in HSV
    lower_path1 = np.array([111,0,250])
    upper_path1 = np.array([160,10,255])

    
    # Threshold the HSV image for paths
    path = cv2.inRange(hsv, lower_path, upper_path).astype(bool)
    path |= cv2.inRange(hsv, lower_path1, upper_path1).astype(bool)
                
    ## define range of rock color in HSV
    lower_rock = np.array([10,150,110])
    upper_rock = np.array([100,255,180])

    # Threshold the HSV image for rocks
    rock = cv2.inRange(hsv, lower_rock, upper_rock).astype(bool)

    
    
    # we can now construct the path and obstacles vector
    # Index the array of zeros with the boolean array and set to 1
    obstacles = np.ones_like(img[:,:,0]) 

    obstacles[path] = 0
    obstacles[rock] = 0
    #obstacles[sky] = 0

    images = [path,obstacles,rock]
    
    # Return the binary images
    return images

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel



# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


def rotate_pix(xpix, ypix, yaw):
    
    # yaw angle is recorded in degrees so first convert to radians
    yaw_rad = yaw * np.pi / 180
    #rotate
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)

    # Return the result  
    return xpix_rotated, ypix_rotated


# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
   
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot/scale))
    ypix_translated = np.int_(ypos + (ypix_rot/scale))
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


def calc_trajectory(x_goal, y_goal, terrain_matrix):
    
    # first construct the heuristic matrix
    hueristic = np.zeros_like(terrain_matrix)
    
    
    
    
    

"""
This is the bulk of the perception, where the magic happens
Basic functionalily is to 
    -input the camera image
    -define source and destination pixels to generate the transformation matrix
    -perform a perspective transformation with generated matrix such that camera image is aligned with an overhead view
    -update the rover object with transformed images
    -shift the perspective transform such that the zeroes align with the robots coordinate system
    -update the rover object with transformed binaries
    -perform color thresholding to seporate
       -the navigatable path
       -the obstacles
       -any rocks if present
    -update the Rover object with thresholded binaries
    -shift the navigatable path binary to polar coordinates for easy computation of steering angle
    -update the Rover object with polar coordinates
    
"""

def color_thresholding_calcs(Rover):
    # Perform perception steps to update Rover()
   
    # NOTE: camera image is coming to you in Rover.img
    
    #Define source and destination points for perspective transform
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    #source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    source = np.float32([[15, 140], [300 ,140],[200, 96], [119, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    

    
    # Apply color threshold to identify navigable terrain/obstacles/rock samples
    #here the return variable threshold_images contains three binaries:
        # threshold_images[:,:,0] is the navigatable path
        # threshold_images[:,:,1] is the obstacles
        # threshold_images[:,:,2] is the rocks
    threshold_images = color_thresh(perspect_transform(Rover.img,source,destination))
   
  
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = np.zeros_like(Rover.img[:,:,0]) + threshold_images[1]*255
    Rover.vision_image[:,:,1] = np.zeros_like(Rover.img[:,:,0]) + threshold_images[2]*255
    Rover.vision_image[:,:,2] = np.zeros_like(Rover.img[:,:,0]) + threshold_images[0]*255
    
    # Convert map image pixel values to rover-centric coords
    rock_x, rock_y = rover_coords(threshold_images[2])
    path_x, path_y = rover_coords(threshold_images[0])
    obstacle_x, obstacle_y = rover_coords(threshold_images[1])
    
    return rock_x,rock_y,path_x,path_y,obstacle_x,obstacle_y
    

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    
    rock_x,rock_y,path_x,path_y,obstacle_x,obstacle_y = color_thresholding_calcs(Rover)
    
    # here I split the color thresholding of from the rest of perception_step to minimize the strain on the stack
    # the return var is 
    # [rock_x,rock_y,path_x,path_y,obstacle_x,obstacle_y]
    
    
    Rover.navigateable_path_pixels = len(path_x)
    
    naviagateable_section_of_interest = path_y[path_x < Rover.x_range]
    
    Rover.offset = Rover.offset_factor * np.median(naviagateable_section_of_interest[naviagateable_section_of_interest > 0])
    
    

    # Convert rover-centric pixel values to world coordinates
    
    rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, Rover.pos[0], Rover.pos[1], Rover.yaw,200,10)
    path_x_world, path_y_world = pix_to_world(path_x, path_y, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_x, obstacle_y, Rover.pos[0], Rover.pos[1], Rover.yaw,200,10)
    
    
    # update the desired position. Here the Rover is intended to function as a wall crawler
    # following the right side wall. The desired position is the median of all y positions less than zero.
    # this should keep the rover close to the right hand wall while ignoring outliers in perception in the y dir
    
    
    # here i perform an averaging of the past median values to minimize influence from spikes in perception
    #des_value = np.median(path_y[path_x < Rover.x_range])
    des_value = np.median(naviagateable_section_of_interest)
    if not (np.isnan(des_value)):
        Rover.avg_y_pos.pop(0)
        Rover.avg_y_pos.append(des_value )
    Rover.des_y_pos = int(round(np.mean(Rover.avg_y_pos)))
    
    
    
    # Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[path_y_world, path_x_world, 2] += 1


   
    
    
 
    
    
    return Rover