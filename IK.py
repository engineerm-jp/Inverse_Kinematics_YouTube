# This code solves a basic inverse kinematics. 
# Note: the output can only be used for a certain configuration of joints. For more detail, please refer to the diagram/s. 
# This code may still contain some bugs so please be be careful as I can not take any responsibility. So please test this code
# thoroughly beforehand by giving a variety of inputs, and confirm that it gives the required output. 
# Also, the input - the coordinates - must be within the reach of the leg, otherwise it will cause math error
# YouTube: https://youtu.be/YCw0JkgeTv8

from math import sin,cos,acos,atan,sqrt,degrees,radians, pi

def coordinate_to_degrees(x, y): # function to convert coordinates to angles from the x-axis (0~360)
    x += 0.00001    
 
    if x >= 0 and y >= 0:   # first quadrant
        angle = degrees(atan(y/x))
    elif x < 0 and y >= 0:  # second quadrant
        angle = 180 + degrees(atan(y/x))
    elif x < 0 and y < 0:   # third quadrant
        angle = 180 + degrees(atan(y/x))
    elif x >= 0 and y < 0:  # forth quadrant
        angle = 360 + degrees(atan(y/x))
    return round(angle,1)


# try the following code to see how it works
#
# test_coordinates = [[0,0], [1,1], [0,1], [-1,1], [-1, 0], [-1,-1], [0, -1], [1,-1]]
# for XY in test_coordinates: 
#     print(f'{XY[0]}, {XY[1]} in degrees is {coordinate_to_degrees(XY[0], XY[1])}')


def IK(pos): 
    # this code calculates the following angles
    # theta1 : this is the angle of the leg from the x-axis about the z-axis
    # theta2 : this is the angle from the x-axis about the y-axis. Note that depending on the configuration of the robot's joint, you may need to
    #          adjust the output. For example, add offset, and/or revert the sign of the angle.
    # theta3 : this is the angle the for the third servo motor and is measured from the axis that is co-linear to the femur. 
 
    x, y, z = pos[0], pos[1], pos[2]
    x += 0.0000001 # this is to avoid zero-division math error

    # specify the length of the leg components
    coxa    = 10      # coxa length
    femur   = 10      # femur length
    tibia   = 10      # tibia length

    theta1 = coordinate_to_degrees(x, y)    # this is the angle from x-axis in anticlockwise rotation

    # remove the offset due to the length of coxa    
    x -= coxa*cos(radians(theta1))
    y -= coxa*sin(radians(theta1))

    #print(x,y)

    P = sqrt(x**2 + y**2)
        
    if P > femur + tibia: # if the goal coordinate is too far away, this makes sure to avoid math. err. 
        P = femur + tibia 
    
    alpha = atan(z/P)

    c = sqrt(P**2 + z**2)
    print(c)

    
    beta = acos((femur**2+c**2-tibia**2)/(2*femur*c))
    theta2 = beta + alpha
    theta3 = acos((tibia**2+femur**2-c**2)/(2*tibia*femur)) - pi
    
    return round(theta1,1), round(degrees(theta2),1), round(degrees(theta3),1)

# Example of usage
angles = IK([7.07, -7.07, 10])
print(angles)
