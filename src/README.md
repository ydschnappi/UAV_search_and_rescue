# How to run inch-by-inch search

1. run unity(Terminal 1)

   ```
   roslaunch unity_bridge unity_sim.launch 
   ```
   
6. run victim(Terminal 2)

   ```
   rosrun unity_bridge victim
   ```

7. run trajectory(Terminal 3)

   ```
   roslaunch basic_waypoint_pkg waypoint_mission.launch
   ```

8. Check if the victim are found

   go back to the terminal of victim(Terminal 2)







