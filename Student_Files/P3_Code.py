import sys
from time import sleep

sys.path.append('../')

from Common.project_library import *

# Modify the information below according to you setup and uncomment the entire section

# 1. Interface Configuration
project_identifier = 'P3B' # enter a string corresponding to P0, P2A, P2A, P3A, or P3B
ip_address = '169.254.45.84' # enter your computer's IP address
hardware = False # True when working with hardware. False when working in the simulation

# 2. Servo Table configuration
short_tower_angle = 270 # enter the value in degrees for the identification tower 
tall_tower_angle = 0 # enter the value in degrees for the classification tower
drop_tube_angle = 180 # enter the value in degrees for the drop tube. clockwise rotation from zero degrees

# 3. Qbot Configuration
bot_camera_angle = -21.5 # angle in degrees between -21.5 and 0

# 4. Bin Configuration
# Configuration for the colors for the bins and the lines leading to those bins.
# Note: The line leading up to the bin will be the same color as the bin 

bin1_offset = 0.13 # offset in meters
bin1_color = [1,0,0] # e.g. [1,0,0] for red
bin2_offset = 0.19
bin2_color = [0,1,0]
bin3_offset = 0.16
bin3_color = [0,0,1]
bin4_offset = 0.17
bin4_color = [0,0,0]

#--------------- DO NOT modify the information below -----------------------------

if project_identifier == 'P0':
    QLabs = configure_environment(project_identifier, ip_address, hardware).QLabs
    bot = qbot(0.1,ip_address,QLabs,None,hardware)
    
elif project_identifier in ["P2A","P2B"]:
    QLabs = configure_environment(project_identifier, ip_address, hardware).QLabs
    arm = qarm(project_identifier,ip_address,QLabs,hardware)

elif project_identifier == 'P3A':
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    configuration_information = [table_configuration,None, None] # Configuring just the table
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
    
elif project_identifier == 'P3B':
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    qbot_configuration = [bot_camera_angle]
    bin_configuration = [[bin1_offset,bin2_offset,bin3_offset,bin4_offset],[bin1_color,bin2_color,bin3_color,bin4_color]]
    configuration_information = [table_configuration,qbot_configuration, bin_configuration]
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
    bins = bins(bin_configuration)
    bot = qbot(0.1,ip_address,QLabs,bins,hardware)
    

#---------------------------------------------------------------------------------
# STUDENT CODE BEGINS
#---------------------------------------------------------------------------------
def container_Dispense():
    """
    Function: container_Dispense()
    Purpose: This function dispenses a container onto the table.
    Inputs: None
    Outputs: None
    Author: Mithunan Ilangainathan(ilangaim) , Junseok Lee(lee509)
    """

    #Casts an random int in the range of container IDs
    cont_ID = random.randint(1,6) 

    #Generates the container and a variable is assigned to its output, which is a list of the container's properties
    cont_Properties = table.dispense_container(cont_ID, True) 
    time.sleep(1)

    #Container's mass and designated bin ID are collected, and then returned. 
    cont_Mass = cont_Properties[1] 
    bin_ID = cont_Properties[2] 
    return bin_ID, cont_Mass 


def container_Load(dropoff):
    """
        Function: container_Load()
        Purpose: This function uses the Q-Arm to move the container from the table to its corresponding
        location on the hopper.
        Inputs: dropoff - coordinates of where on the hopper that the container needs to be dropped off.
        Outputs: None
        Author: Mithunan Ilangainathan (ilangaim), Junseok Lee (lee509)
        """
    PICK_UP = [0.63, 0, 0.22]
    
    #intermediate location arm goes to in order to minimize collision between Q-Arm and hopper during loading
    HOME = [0.406,0,0.483] 
    GRIPPING_ANGLE = 40
    
    #Open/close grip tracker
    grip = 0 
    
    #Order of locations for arm to visit are listed
    key_locations = [PICK_UP, HOME, dropoff] 

    # Arm goes to every key location and wait 2 seconds. Dropoff and pickup spots have an even index value.
    # Afterwards, the bottle is pushed further into the hopper to make space for more bottles (see adjust_bottles function).
    for i in range(len(key_locations)):
        arm.move_arm(*key_locations[i])
        sleep(2)
        if i % 2 == 0:
            arm.control_gripper(pow(-1, grip) * GRIPPING_ANGLE)
            grip += 1
            sleep(2)
    safe_return()
    adjust_bottles(dropoff[1], dropoff[0])

def safe_return():
    """
    Function: safe_return()
    Purpose: This function moves the Q-Arm out of the way of the hopper
    safely.
    Inputs: None
    Outputs: None
    Author: Mithunan Ilangainathan (ilangaim)
    """
    # Arm is rotated upwards fully to avoid bottles, and then is rotated
    # away from the direction of the hopper before going back to
    # home position.
    arm.rotate_elbow(-15)
    arm.rotate_shoulder(-15)
    sleep(2)
    arm.home()


def adjust_bottles(end_pos, offset=0.02):
    """
    Function: adjust_bottles()
    Purpose: This function pushes bottles currently on the hopper back to
    allow for more space for another bottle to be placed.
    Inputs: end_pos - y-coordinate of dropoff location; offset - x-coordinate of dropoff location
    Outputs: None
    Author: Mithunan Ilangainathan (ilangaim)
    """
    # Calculates and stores, with a list, the push start and end locations based on the
    # specificed offset and finishing positions.
    start = (offset, -0.3, 0.45)
    drop = (offset, end_pos + 0.02, 0.45)
    locations = [start, drop]

    # Gripper is closed fully to fit inside the hopper. Arm goes to the start
    # position, and then goes to the drop location to push the bottles
    # further inside the hopper. Arm is then opened slightly o prevent any accidental grips
    # in the virtual environment. Arm is moved away from the hopper
    # and goes back to its home position.
    sleep(2)
    arm.control_gripper(45)
    sleep(2)
    for i in range(2):
        # Start coordinate --> drop coordinate
        arm.move_arm(*locations[i]) 
        sleep(2)
    arm.control_gripper(-13)
    sleep(1)
    safe_return()

def container_Transfer(bin_ID):
    """
    Function: container_Transfer(bin_ID)
    Purpose: This function uses the ultrasonic sensor to transfer containers to their corresponding bin.
    Inputs: bin_ID - string
    Outputs: None
    Author: Mithunan Ilangainathan(ilangaim), Junseok Lee(lee509)
    """

    """ Passing distance constant is declared. When the hopper is going away
    from a bin and its separation distance reaches 0.15 m or more, the hopper
    won't be encountering that bin anymore (more info on this below).
    """
    PASSING_DIST = 0.15

    #Ultrasonic sensor activated.
    #The bot must be within a distance of 0.09 of each bin to encounter that bin.
    #bin_num is the Bin Number of a particular bin.
    bot.activate_ultrasonic_sensor()
    if bin_ID == 'Bin01':
        print("Target bin:" , bin_ID)
        bin_Target = 0.09
        bin_num = 1
    elif bin_ID == 'Bin02':
        print("Target bin:" , bin_ID)
        bin_Target = 0.09
        bin_num = 2
    elif bin_ID == 'Bin03':
        print("Target bin:" , bin_ID)
        bin_Target = 0.09
        bin_num = 3
    elif bin_ID == 'Bin04':
        print("Target bin:" , bin_ID)
        bin_Target = 0.09
        bin_num = 4
    else:
        print("Invalid bin input")
        
    #This variable stores the distance between the bot and a nearby bin.
    bin_Dist = bot.read_ultrasonic_sensor()

    #Encountered bin status defaulted to false, and cur_bin is the latest bin encountered before dumping the containers.
    encountered_bin = False
    cur_bin = 0
    
    #Line is followed until the Q-bot approaches the correct bin.
    while cur_bin < bin_num: 
        bot.line_following_sensors()
        if bot.line_following_sensors() == [1,1]:
            bot.set_wheel_speed([0.08,0.08])
        elif bot.line_following_sensors() == [1,0]:
            bot.set_wheel_speed([0.04,0.08])
        elif bot.line_following_sensors() == [0,1]:
            bot.set_wheel_speed([0.08,0.04])
        elif bot.line_following_sensors() != [1,1]:
            bot.set_wheel_speed([-0.04,0.04])
        
        #Distance from nearby bin is always being checked.
        bin_Dist = bot.read_ultrasonic_sensor()
        print(bin_Dist)
        """ If the bot is within approaching distance of a bin and it hasn't
        already encountered it, this means that it has encountered a new bin."""
        if bin_Dist <= bin_Target and not encountered_bin:
            cur_bin +=1
            encountered_bin = True
        
        #Otherwise if it's already encountered a specific bin and has gotten further away from it,
        #the encounter status is set to false, as it is not encountering that bin anymore.
        elif encountered_bin and bin_Dist > PASSING_DIST:
            encountered_bin = False
        print(cur_bin, "bins encountered.")

    #Otherwise, once the correct bin to dump the containers is reached,
    #go forward for a couple of seconds, stop the bot and trigger the deposit function.
    else:
        if bot.line_following_sensors() == [1,1]:
            bot.set_wheel_speed([0.025, 0.025])
            bot.forward_time(2)
        elif bot.line_following_sensors() == [0,1]:
            bot.set_wheel_speed([0.025,0])
            bot.forward_time(2)
        elif bot.line_following_sensors() == [1,0]:
            bot.set_wheel_speed([0,0.025])
            bot.forward_time(2)
        elif bot.line_following_sensors() != [1,1]:
            bot.set_wheel_speed([-0.025,0.025])
            bot.forward_time(2)

    bot.stop()
    bot.deactivate_ultrasonic_sensor()
    return bin_Dist

def container_Deposit(bin_Dist):
    """
    Function: container_Deposit(bin_Dist)
    Purpose: This function is to deposit containers to the bin
    Inputs: bin_Dist - float that represents the distance between the Q-Bot and a nearby bin in metres.
    Outputs: None
    Author: Mithunan Ilangainathan(ilangaim), Junseok Lee(lee509)
    """ 
    bot.activate_stepper_motor()
    bot.dump()
    time.sleep(2)
    bot.deactivate_stepper_motor()
    
 
def bot_return_home(home_position):
    """
    Function: bot_return_home(home_position)
    Purpose: Q-bot returns to the home position.
    Inputs: home_position - coordinates of home position where Q-Bot is ready for the next cycle.
    Outputs: None
    Author: Mithunan Ilangainathan(ilangaim)
    """
    #home_position coordinates, taken from main function, are rounded to only one decimal
    home_position = [round(num, 1) for num in home_position] 

    #While loop is executed until both has reached close to home.
    while True: 
        bot.line_following_sensors()
        if bot.line_following_sensors() == [1,1]:
            bot.set_wheel_speed([0.08,0.08])
        elif bot.line_following_sensors() == [1,0]:
            bot.set_wheel_speed([0.04,0.08])
        elif bot.line_following_sensors() == [0,1]:
            bot.set_wheel_speed([0.08,0.04])
        elif bot.line_following_sensors() != [1,1]:
            bot.set_wheel_speed([-0.04,0.04])
            
        #bot_position variable is set as the coordinates of bot position.
        bot_position = bot.position() 

        # Bot's coordinates are rounded to only one decimal
        bot_position = [round(bot_position[0], 1),round(bot_position[1], 1), round(bot_position[2], 1)]
        print(bot_position, home_position, (bot_position == home_position))

        """ Once the bot has reached close to it's home position, it rotates and moves
        forward a couple times to ensure it is essentially in it's home position. 
        """
        if bot_position == home_position:
            bot.rotate(-95)
            bot.forward_time(0.4)
            bot.rotate(95)
            bot.forward_time(0.3455)
            bot.stop()
            bot.rotate(2)
            break
    return

def main():
    """
    Function: main()
    Purpose: Includes all necessary functions for infinite cycles
    Inputs: None
    Outputs: None
    Author: Mithunan Ilangainathan(ilangaim), Junseok Lee(lee509)
    """ 

    #Info and status of leftover container on table is initialized.
    turntable_container = []
    leftover_container = False

    #Infinite loop of entire sorting and recycling system.
    while True: 
        #Bot moves forward for an extremely short distance to help with alignment of bot.
        bot.forward_time(0.003)

        #Bot's home position is calculated and printed.
        home_pos = bot.position() 
        print(home_pos)

        #dropoff location for first, second, and third containers are listed
        dropoff_Locations = [[0.02,-0.58,0.55],[0.02, -0.50, 0.55],[0.02,-0.44,0.55]]

        #List of bin IDs and total mass of containers dispensed are initialized.
        bin_List = []
        total_Mass = 0

        #Each cycle has the potential to dispense and load a max of 3 containers.
        for cont_Count in range (3):

            """ If the first container in a cycle was a leftover container from the previous
            cycle, then that container's bin ID will be the target location of the bot. The
            total mass of the containers is started off with the mass of the leftover container.
            The leftover container is immediately loaded and the for loop is advanced to the next
            iteration.
            """
            if leftover_container == True:
                bin_ID = turntable_container[0]
                bin_List.append(bin_ID)
                total_Mass = turntable_container[1]
                print("Bin Target:", bin_ID)
                print("Current Load: ", total_Mass)
                container_Load(dropoff_Locations[cont_Count])

                #leftover container info is reset to default settings
                turntable_container = []
                leftover_container = False
                continue
            
            #Dispense function is executed.
            container_Properties = container_Dispense() 
            
            #Bin ID of dispensed container is collected from the properties list.
            bin_ID = container_Properties[0] 

            #Bin ID is appended into list of bin IDs.
            bin_List.append(bin_ID) 
            
            #Mass of dispensed container is collected from the properties list.
            cont_Mass = container_Properties[1] 
            print("Bin Target:", bin_ID)

            """If the bin ID of the current container does not match the bin ID of the first
            container, mismatch message pops up, current container becomes a leftover container, and the for
            loop is exited.
            """
            if bin_List[0] != bin_ID:
                print("Mismatch bin ID!", bin_ID, "does not match", bin_List[0])
                turntable_container.append(bin_ID)
                turntable_container.append(cont_Mass)
                bin_ID = bin_List[0]
                leftover_container = True
                break
            
            #Current container mass is added on to the total mass of containers.
            total_Mass += cont_Mass 
            print("Current Load: ", total_Mass)

            """If the total mass of the containers exceed 90 g, exceed mass limit message pops up,
            current container becomes a leftover container, and the for loop is exited.
            """
            if total_Mass > 90:
                print("Total mass of",round(total_Mass,2), "exceeds 90 g limit.")
                turntable_container.append(bin_ID)
                turntable_container.append(cont_Mass)
                leftover_container = True
                break

            #Each container is loaded to their respective dropoff locations.
            container_Load(dropoff_Locations[cont_Count])

        """ The bot moves along the linepath to the assigned bin, deposits the containers, and returns
        home to complete a cycle.
        """
        sensor_Readings = container_Transfer(bin_ID)
        bin_Dist = sensor_Readings
        container_Deposit(bin_Dist)
        bot_return_home(home_pos)
        print("Cycle Completed!")
        time.sleep(0.5)

#main function is executed.
main() 
    
#---------------------------------------------------------------------------------
# STUDENT CODE ENDS
#---------------------------------------------------------------------------------