/home/dan/learn/MA_Sentry/src/rm_control/rm_decision/src/random_explore.cc:29:14: error: ‘geometry_msgs::PointStamped’ {aka ‘struct geometry_msgs::PointStamped_<std::allocator<void> >’} has no member named ‘x’
   29 |     new_goal.x = cos(angle) * radius;
      |              ^
/home/dan/learn/MA_Sentry/src/rm_control/rm_decision/src/random_explore.cc:30:14: error: ‘geometry_msgs::PointStamped’ {aka ‘struct geometry_msgs::PointStamped_<std::allocator<void> >’} has no member named ‘y’
   30 |     new_goal.y = sin(angle) * radius;
      |              ^
make[2]: *** [CMakeFiles/rm_decision_test_node.dir/build.make:89: CMakeFiles/rm_decision_test_node.dir/src/random_explore.cc.o] Error 1
make[2]: *** Waiting for unfinished jobs....
make[1]: *** [CMakeFiles/Makefile2:784: CMakeFiles/rm_decision_test_node.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
cd /home/dan/learn/MA_Sentry/build/rm_decision; catkin build --get-env rm_decision | catkin env -si  /usr/bin/make --jobserver-auth=3,4; cd -

...............................................................................
Failed     << rm_decision:make                       [ Exited with code 2 ]    
Failed    <<< rm_decision                            [ 3.1 seconds ]           
Abandoned <<< icp_localization                       [ Unrelated job failed ]  
Finished  <<< boundary_handler                       [ 0.5 seconds ]           
Finished  <<< far_planner                            [ 0.7 seconds ]           
Finished  <<< pointmatcher_ros                       [ 0.4 seconds ]           
Finished  <<< graph_decoder                          [ 0.7 seconds ]           
[build] Summary: 36 of 38 packages succeeded.                                  
[build]   Ignored:   None.                                                     
[build]   Warnings:  None.                                                     
[build]   Abandoned: 1 packages were abandoned.                                
[build]   Failed:    1 packages failed.                                        
[build] Runtime: 4.4 seconds total.           
