# Behavior LookAtPoint
Paczka tworzy behavior używany przez Aerostack (oprogramowanie grupy Vision4UAV: https://github.com/Vision4UAV/Aerostack)
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
    
4. Edytować plik `behavior_catalog.yaml`. Plik znajduje się w lokalizacji: `~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/drone1` 
    W sekcji `behavior_descriptors` należy dokleić poniższe linie:
	    
		
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
`coordinates=[x,y,z]`
lub 
`relative_coordinates=[x,y,z]`
Przykład wywołania:
`result = api.executeBehavior('LOOK_AT_POINT', coordinates=[4, 6.5, 3])`