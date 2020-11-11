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
sleep 10

gnome-terminal --tab --command="rosrun multi_robot test_d2.py" \
--tab --command="rosrun multi_robot test_d1.py"
sleep 20

gnome-terminal --tab --command="rosrun multi_robot test_d4.py" \
--tab --command="rosrun multi_robot test_d9.py"

sleep 10

gnome-terminal --tab --command="rosrun multi_robot test_d5.py" \
--tab --command="rosrun multi_robot test_d8.py" 

sleep 5
gnome-terminal --tab --command="rosrun multi_robot voronoi.py"

sleep 5
gnome-terminal \
--tab --command="rosrun multi_robot test_d10.py"  --tab --command="rosrun multi_robot test_d11.py" \
--tab --command="rosrun multi_robot test_d12.py" --tab --command="rosrun multi_robot test_d13.py" \
--tab --command="rosrun multi_robot test_d14.py" --tab --command="rosrun multi_robot test_d15.py" \
--tab --command="rosrun multi_robot test_d16.py" --tab --command="rosrun multi_robot test_d17.py" \
--tab --command="rosrun multi_robot test_d3.py"  

sleep 30
gnome-terminal --tab --command="rosrun multi_robot test_d6.py" --tab --command="rosrun multi_robot test_d7.py" 

