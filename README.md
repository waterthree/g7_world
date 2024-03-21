the custom pacakge of Group 7 for robotics assignment(3-4)



Put this project under src folder, and change dir to upper folder, then use colcon to compile the package.

/* clean the workspace
   delete folders which been built previously */
colcon clean workspace

/* build all the packages under src folder */
colcon build --symlink-install
or
/* only build g7_world */
colcon build --packages-select g7_world
