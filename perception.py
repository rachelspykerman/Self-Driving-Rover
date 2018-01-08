import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def obstacles(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    obstacle = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    obstacle[below_thresh] = 1
    # Return the binary image
    return obstacle

def rocks(img, rgb_thresh=(110, 110, 50)):
    # Require that each pixel be above all three threshold values in RGB
    # rock will now contain a boolean array with "True"
    # where threshold was met
    rock = ((img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2]))
    
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    
    # Index the array of zeros with the boolean array and set to 1
    color_select[rock] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
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

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
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


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img

    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    groundSel = color_thresh(warped)
    obstacleSel = obstacles(warped)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacleSel*255
    Rover.vision_image[:,:,2] = groundSel*255
    
    # 5) rover centric coordinates
    xpix_ground, ypix_ground = rover_coords(groundSel)
    xpix_obstacle, ypix_obstacle = rover_coords(obstacleSel)
    scale = dst_size * 2
    rover_xpos, rover_ypos = Rover.pos
    yaw = Rover.yaw
    worldshape = Rover.worldmap.shape[0]
    
    # coordinates in the world map
    x_ground_world, y_ground_world = pix_to_world(xpix_ground, ypix_ground, rover_xpos, rover_ypos, yaw, worldshape, scale)    
    x_obstacle_world, y_obstacle_world = pix_to_world(xpix_obstacle, ypix_obstacle, rover_xpos, rover_ypos, yaw, worldshape, scale)
    
    # each time we find an obstacle we add 1 to the red channel, if we find a lot of obstacles in one pixel, you deem it an obstacle
    Rover.worldmap[y_obstacle_world, x_obstacle_world, 0] += 1
    # each time we find a pixel of navigable terrain we add 10 to the blue channel, so we favor anywhere we find navigable terrain 
    Rover.worldmap[y_ground_world, x_ground_world, 2] += 10
    
    dist, angles = to_polar_coords(xpix_ground, ypix_ground)
    Rover.nav_angles = angles
    Rover.nav_dists = dist
    
    rocksSel = rocks(warped)
    if rocksSel.any():
        # rover centric coordinates
        xpix_rocks, ypix_rocks = rover_coords(rocksSel)
        # coordinates in the world map
        x_rock_world, y_rock_world = pix_to_world(xpix_rocks, ypix_rocks, rover_xpos, rover_ypos, yaw, worldshape, scale)
        rock_dist, rock_ang = to_polar_coords(xpix_rocks, ypix_rocks)
        # get rock closest to rover...get the minimum distance rock pixel, and make that the center point
        rock_idx = np.argmin(rock_dist)
        rock_xcen = x_rock_world[rock_idx]
        rock_ycen = y_rock_world[rock_idx]
        Rover.worldmap[rock_ycen, rock_xcen, 1] = 255
        Rover.vision_image[:,:,1] = rocksSel*255
        if((len(x_rock_world) > 0) and (len(y_rock_world) > 0)):
            Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_rocks, ypix_rocks)
            Rover.mode = "pickup"
            print("Rover.mode " + str(Rover.mode))
    else:
        Rover.vision_image[:, :, 1] = 0
        if Rover.mode == "pickup":
            Rover.mode = 'forward'
        print("Rover.mode " + str(Rover.mode))
 
    
    
    return Rover