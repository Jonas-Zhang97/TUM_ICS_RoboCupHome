# Tutorial 3
## Prework:
 Paste the folders in zip into your tiago workspace, if there are same file, please click replace. build the whole workspace 
 Source the setup.bash in this workspace when opening a new terminal.
 
## Exercise 1:  Generate maps for two worlds using Tiago / HSRB
robot:tiago

These maps are in world folder. Please put them into $HOME/.pal/tiago_maps/configurations/

COMMAND TO TEST WORLD:
```
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true lost:=true map:=$HOME/.pal/tiago_maps/configurations/default_world
```

```
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true lost:=true map:=$HOME/.pal/tiago_maps/configurations/tutorial_office
```
## Exercise 2: Localization
robot:tiago 

COMMAND TO TEST:

In first terminal:
```
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true lost:=true map:=$HOME/.pal/tiago_maps/configurations/default_world
```
In second terminal:
```
roslaunch tiago_localization tiago_localization.launch
```
**note: the whole process will last serval seconds. Please wait. The sencond terminal will automatically stop when it finishes

## Exercise 3: Navigate the Map and Exercise 4: Planning in Cartesian space with MoveIt!
robot:tiago

COMMAND TO TEST:

In the first terminal:
```
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true
```
In the second terminal: 
```
roslaunch tiago_navigation tiago_localization.launch
```
**note: While tiago from point A goes to point B, it will get lost. Because the path is too narrow. It will automatically replan the path. Please wait.


## Credits

This Homework was developed by [Zhen Chen] and [Yueyang Zhang]
