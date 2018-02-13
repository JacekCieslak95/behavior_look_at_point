# Behavior LookAtPoint

Read in [Polish]

This package creates behavior used by [Aerostack] (framework developed by [Vision4UAV]).
This behavior makes the drone face a given point with its front.

### Instalation ###
1. Move the files of this repository into the
    `~/workspace/ros/aerostack_catkin_ws/src/`
   folder so that the catalogue tree looks as follows:
    
        ~/workspace/ros/aerostack_catkin_ws/src/
            -behavior_look_at_point
    		    -CMakeLists.txt
                -package.xml
                -launch
                    -behavior_look_at_point.launch
    			-src
                    -include
                        -behavior_look_at_point.h
                    -source
                        -behavior_look_at_point.cpp
                        -behavior_look_at_point_main.cpp

2. Compile using catkin_make `~/workspace/ros/aerostack_catkin_ws/$ catkin_make`
3. Edit `simulated_quadrotor_basic.sh` file - in the startup script paste the following lines at the end:
    
	    `#----------------------------------------------` \
	    `# Behavior LookAtPoint                                    ` \
	    `#----------------------------------------------` \
	    --tab --title "Behavior LookAtPoint" --command "bash -c \"
	    roslaunch behavior_look_at_point behavior_look_at_point.launch --wait \
    		drone_id_namespace:=drone$NUMID_DRONE \
    		drone_id:=$NUMID_DRONE \
    		my_stack_directory:=${AEROSTACK_STACK};
    	exec bash\"" \
    
4. Edit `behavior_catalog.yaml` file located in `~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/droneX` 
    Go to the `behavior_descriptors` section and paste the following lines:
#### NOTICE! It should be pasted in the `configs/droneX` folder of every drone that is going to utilize this behavior
		
          - behavior: LOOK_AT_POINT
		    incompatible_lists: [motion_behaviors]
            capabilities: [SETPOINT_BASED_FLIGHT_CONTROL]
			arguments:
			  - argument: ANGLE
			    allowed_values: [-360,360]
		      - argument: POSITION
			    allowed_values: [-100,100]
				dimensions: 3
			  - argument: RELATIVE_COORDINATES
			    allowed_values: [-100,100]
			    dimensions: 3

##### NOTICE! Make the indentation using the space bar, not the tab key!

### Arguments taken: ###
Behavior takes following arguments:
    
    coordinates=[x,y,z]
    
or
    
    relative_coordinates=[x,y,z]
    
This sets the point the drone will face.
    
    angle=x
    
It is the angle (in degrees) at which the drone will be angled towards the destination point.

Example of a call::
`result = api.executeBehavior('LOOK_AT_POINT', coordinates=[4, 6.5, 3], angle=15)`


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)
   [Polish]: <https://github.com/JacekCieslak95/behavior_look_at_point/blob/master/README.md>
   [English]: <https://github.com/JacekCieslak95/behavior_look_at_point/blob/master/README_en.md>
   [Aerostack]: <https://github.com/Vision4UAV/Aerostack>
   [Vision4UAV]: <https://github.com/Vision4UAV>
