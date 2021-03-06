# Behavior LookAtPoint

Read in [English]

Paczka tworzy behavior używany przez [Aerostack] (oprogramowanie grupy [Vision4UAV])
Behavior LookAtPoint powoduje, że dron obraca się frontem w kierunku zadanego punktu
### Instalacja ###
1. Pliki niniejszego repozytorium należy umieścić w folderze 
    `~/workspace/ros/aerostack_catkin_ws/src/`
    tak, aby tworzyły poniższe drzewo:
    
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

2. Przeprowadzić kompilację catkin `~/workspace/ros/aerostack_catkin_ws/$ catkin_make`
3. Edytować plik `simulated_quadrotor_basic.sh` - W skrypcie uruchamiającym należy dokleić na końcu poniższe linie:
    
	    `#----------------------------------------------` \
	    `# Behavior LookAtPoint                                    ` \
	    `#----------------------------------------------` \
	    --tab --title "Behavior LookAtPoint" --command "bash -c \"
	    roslaunch behavior_look_at_point behavior_look_at_point.launch --wait \
    		drone_id_namespace:=drone$NUMID_DRONE \
    		drone_id:=$NUMID_DRONE \
    		my_stack_directory:=${AEROSTACK_STACK};
    	exec bash\"" \
    
4. Edytować plik `behavior_catalog.yaml`. Plik znajduje się w lokalizacji: `~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/droneX` 
    W sekcji `behavior_descriptors` należy dokleić poniższe linie:
#### UWAGA! Należy to wkleić do folderu `configs/droneX` każdego drona, którego chcemy uruchamiać z danym zachowaniem.
#### Np. Używając tego w dronach 1 i 2 poniższy fragment należy dokleić do `behavior_catalog.yaml` w folderach `configs/drone1` oraz `configs/drone2`
	    
		
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
				
##### UWAGA! Wcięcia powinny być realizowane przez spacje, nie tabulatory!

### Przyjmowane argumenty ###
Behavior przyjmuje argumenty:
    
    coordinates=[x,y,z]
    
lub
    
    relative_coordinates=[x,y,z]
    
Jest to punkt na który dron będzie skierowany.
    
    angle=x
    
Jest to kąt (w stopniach) pod jakim dron będzie skierowany względem punktu.

Przykład wywołania:
`result = api.executeBehavior('LOOK_AT_POINT', coordinates=[4, 6.5, 3], angle=15)`

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)
   [Polish]: <https://github.com/JacekCieslak95/behavior_look_at_point/blob/master/README.md>
   [English]: <https://github.com/JacekCieslak95/behavior_look_at_point/blob/master/README_en.md>
   [Aerostack]: <https://github.com/Vision4UAV/Aerostack>
   [Vision4UAV]: <https://github.com/Vision4UAV>