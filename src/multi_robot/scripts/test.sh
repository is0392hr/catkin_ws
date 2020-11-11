#!/bin/sh
rosservice call /drone01/enable_motors true
rosservice call /drone02/enable_motors true
rosservice call /drone03/enable_motors true
rosservice call /drone04/enable_motors true
rosservice call /drone05/enable_motors true
rosservice call /drone06/enable_motors true
rosservice call /drone07/enable_motors true
rosservice call /drone08/enable_motors true
rosservice call /drone09/enable_motors true
rosservice call /drone10/enable_motors true
rosservice call /drone11/enable_motors true
rosservice call /drone12/enable_motors true
rosservice call /drone13/enable_motors true
rosservice call /drone14/enable_motors true
rosservice call /drone15/enable_motors true
rosservice call /drone16/enable_motors true
rosservice call /drone17/enable_motors true
rosservice call /drone18/enable_motors true
rosservice call /drone19/enable_motors true
rosservice call /drone20/enable_motors true
rosservice call /drone21/enable_motors true
rosservice call /drone22/enable_motors true
rosservice call /drone23/enable_motors true
rosservice call /drone24/enable_motors true
rosservice call /drone25/enable_motors true
rosservice call /drone26/enable_motors true
rosservice call /drone27/enable_motors true
rosservice call /drone28/enable_motors true
rosservice call /drone29/enable_motors true
rosservice call /drone30/enable_motors true
rosservice call /drone31/enable_motors true
rosservice call /drone32/enable_motors true
rosservice call /drone33/enable_motors true
rosservice call /drone34/enable_motors true
rosservice call /drone35/enable_motors true
rosservice call /drone36/enable_motors true
rosservice call /drone37/enable_motors true

gnome-terminal --tab --command="rosrun multi_robot test_10.py" --tab --command="rosrun multi_robot test_20.py"  --tab --command="rosrun multi_robot test_30.py" --tab --command="rosrun multi_robot test_40.py"  

# gnome-terminal --tab --command="rosrun multi_robot test_d2.py" --tab --command="rosrun multi_robot test_d1.py"  --tab --command="./graph.py"  
# sleep 20
# gnome-terminal --tab --command="rosrun multi_robot test_d4.py" --tab --command="rosrun multi_robot test_d9.py"
# sleep 10
# # gnome-terminal --tab --command="rosrun multi_robot test_d3.py" 
# # sleep 5
# # gnome-terminal --tab --command="rosrun multi_robot test_d10.py" --tab --command="rosrun multi_robot test_d11.py"
# # sleep 5
# # gnome-terminal --tab --command="rosrun multi_robot test_d12.py" --tab --command="rosrun multi_robot test_d13.py"
# # sleep 10
# gnome-terminal --tab --command="rosrun multi_robot test_d5.py" --tab --command="rosrun multi_robot test_d8.py" 
# sleep 30
# gnome-terminal --tab --command="rosrun multi_robot test_d6.py" --tab --command="rosrun multi_robot test_d7.py" 