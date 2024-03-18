Errors     << rm_decision:make /home/dan/learn/MA_Sentry/logs/rm_decision/build.make.099.log
/usr/bin/ld: CMakeFiles/rm_decision_test_node.dir/src/random_explore.cc.o: in function `RandomExplore::RandomExplore(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, BT::NodeConfig const&)':
random_explore.cc:(.text+0x74): undefined reference to `BT::SyncActionNode::SyncActionNode(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, BT::NodeConfig const&)'
collect2: error: ld returned 1 exit status
make[2]: *** [CMakeFiles/rm_decision_test_node.dir/build.make:142: /home/dan/learn/MA_Sentry/devel/.private/rm_decision/lib/rm_decision/rm_decision_test_node] Error 1
make[1]: *** [CMakeFiles/Makefile2:784: CMakeFiles/rm_decision_test_node.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
cd /home/dan/learn/MA_Sentry/build/rm_decision; catkin build --get-env rm_decision | catkin en
