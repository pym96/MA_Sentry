rrors     << rm_decision:make /home/dan/learn/MA_Sentry/logs/rm_decision/build.make.135.log
In file included from /usr/include/boost/function/detail/maybe_include.hpp:22,
                 from /usr/include/boost/function/detail/function_iterate.hpp:14,
                 from /usr/include/boost/preprocessor/iteration/detail/iter/forward1.hpp:53,
                 from /usr/include/boost/function.hpp:71,
                 from /opt/ros/noetic/include/ros/forwards.h:40,
                 from /opt/ros/noetic/include/ros/common.h:37,
                 from /opt/ros/noetic/include/ros/ros.h:43,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/include/rm_decision/random_explore.hpp:5,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/src/random_explore.cc:1:
/usr/include/boost/function/function_template.hpp: In instantiation of ‘static void boost::detail::function::function_void_mem_invoker1<MemberPtr, R, T0>::invoke(boost::detail::function::function_buffer&, T0) [with MemberPtr = void (RandomExplore::*)(const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&); R = void; T0 = const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&]’:
/usr/include/boost/function/function_template.hpp:931:38:   required from ‘void boost::function1<R, T1>::assign_to(Functor) [with Functor = void (RandomExplore::*)(const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&); R = void; T0 = const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&]’
/usr/include/boost/function/function_template.hpp:720:7:   required from ‘boost::function1<R, T1>::function1(Functor, typename boost::enable_if_<(! boost::is_integral<Functor>::value), int>::type) [with Functor = void (RandomExplore::*)(const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&); R = void; T0 = const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&; typename boost::enable_if_<(! boost::is_integral<Functor>::value), int>::type = int]’
/usr/include/boost/function/function_template.hpp:1068:16:   required from ‘boost::function<R(T0)>::function(Functor, typename boost::enable_if_<(! boost::is_integral<Functor>::value), int>::type) [with Functor = void (RandomExplore::*)(const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&); R = void; T0 = const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&; typename boost::enable_if_<(! boost::is_integral<Functor>::value), int>::type = int]’
/home/dan/learn/MA_Sentry/src/rm_control/rm_decision/src/random_explore.cc:12:104:   required from here
/usr/include/boost/function/function_template.hpp:230:11: error: no match for call to ‘(boost::_mfi::mf1<void, RandomExplore, const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&>) (const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&)’
  230 |           BOOST_FUNCTION_RETURN(boost::mem_fn(*f)(BOOST_FUNCTION_ARGS));
      |           ^~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/boost/bind/mem_fn.hpp:215,
                 from /usr/include/boost/mem_fn.hpp:22,
                 from /usr/include/boost/function/detail/prologue.hpp:18,
                 from /usr/include/boost/function.hpp:30,
                 from /opt/ros/noetic/include/ros/forwards.h:40,
                 from /opt/ros/noetic/include/ros/common.h:37,
                 from /opt/ros/noetic/include/ros/ros.h:43,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/include/rm_decision/random_explore.hpp:5,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/src/random_explore.cc:1:
/usr/include/boost/bind/mem_fn_template.hpp:163:7: note: candidate: ‘R boost::_mfi::mf1<R, T, A1>::operator()(T*, A1) const [with R = void; T = RandomExplore; A1 = const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&]’
  163 |     R operator()(T * p, A1 a1) const
      |       ^~~~~~~~
/usr/include/boost/bind/mem_fn_template.hpp:163:7: note:   candidate expects 2 arguments, 1 provided
/usr/include/boost/bind/mem_fn_template.hpp:168:25: note: candidate: ‘template<class U> R boost::_mfi::mf1<R, T, A1>::operator()(U&, A1) const [with U = U; R = void; T = RandomExplore; A1 = const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&]’
  168 |     template<class U> R operator()(U & u, A1 a1) const
      |                         ^~~~~~~~
/usr/include/boost/bind/mem_fn_template.hpp:168:25: note:   template argument deduction/substitution failed:
In file included from /usr/include/boost/function/detail/maybe_include.hpp:22,
                 from /usr/include/boost/function/detail/function_iterate.hpp:14,
                 from /usr/include/boost/preprocessor/iteration/detail/iter/forward1.hpp:53,
                 from /usr/include/boost/function.hpp:71,
                 from /opt/ros/noetic/include/ros/forwards.h:40,
                 from /opt/ros/noetic/include/ros/common.h:37,
                 from /opt/ros/noetic/include/ros/ros.h:43,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/include/rm_decision/random_explore.hpp:5,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/src/random_explore.cc:1:
/usr/include/boost/function/function_template.hpp:230:11: note:   candidate expects 2 arguments, 1 provided
  230 |           BOOST_FUNCTION_RETURN(boost::mem_fn(*f)(BOOST_FUNCTION_ARGS));
      |           ^~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/boost/bind/mem_fn.hpp:215,
                 from /usr/include/boost/mem_fn.hpp:22,
                 from /usr/include/boost/function/detail/prologue.hpp:18,
                 from /usr/include/boost/function.hpp:30,
                 from /opt/ros/noetic/include/ros/forwards.h:40,
                 from /opt/ros/noetic/include/ros/common.h:37,
                 from /opt/ros/noetic/include/ros/ros.h:43,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/include/rm_decision/random_explore.hpp:5,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/src/random_explore.cc:1:
/usr/include/boost/bind/mem_fn_template.hpp:176:25: note: candidate: ‘template<class U> R boost::_mfi::mf1<R, T, A1>::operator()(const U&, A1) const [with U = U; R = void; T = RandomExplore; A1 = const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&]’
  176 |     template<class U> R operator()(U const & u, A1 a1) const
      |                         ^~~~~~~~
/usr/include/boost/bind/mem_fn_template.hpp:176:25: note:   template argument deduction/substitution failed:
In file included from /usr/include/boost/function/detail/maybe_include.hpp:22,
                 from /usr/include/boost/function/detail/function_iterate.hpp:14,
                 from /usr/include/boost/preprocessor/iteration/detail/iter/forward1.hpp:53,
                 from /usr/include/boost/function.hpp:71,
                 from /opt/ros/noetic/include/ros/forwards.h:40,
                 from /opt/ros/noetic/include/ros/common.h:37,
                 from /opt/ros/noetic/include/ros/ros.h:43,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/include/rm_decision/random_explore.hpp:5,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/src/random_explore.cc:1:
/usr/include/boost/function/function_template.hpp:230:11: note:   candidate expects 2 arguments, 1 provided
  230 |           BOOST_FUNCTION_RETURN(boost::mem_fn(*f)(BOOST_FUNCTION_ARGS));
      |           ^~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/boost/bind/mem_fn.hpp:215,
                 from /usr/include/boost/mem_fn.hpp:22,
                 from /usr/include/boost/function/detail/prologue.hpp:18,
                 from /usr/include/boost/function.hpp:30,
                 from /opt/ros/noetic/include/ros/forwards.h:40,
                 from /opt/ros/noetic/include/ros/common.h:37,
                 from /opt/ros/noetic/include/ros/ros.h:43,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/include/rm_decision/random_explore.hpp:5,
                 from /home/dan/learn/MA_Sentry/src/rm_control/rm_decision/src/random_explore.cc:1:
/usr/include/boost/bind/mem_fn_template.hpp:184:7: note: candidate: ‘R boost::_mfi::mf1<R, T, A1>::operator()(T&, A1) const [with R = void; T = RandomExplore; A1 = const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > >&]’
  184 |     R operator()(T & t, A1 a1) const
      |       ^~~~~~~~
/usr/include/boost/bind/mem_fn_template.hpp:184:7: note:   candidate expects 2 arguments, 1 provided
make[2]: *** [CMakeFiles/rm_decision_test_node.dir/build.make:89: CMakeFiles/rm_decision_test_node.dir/src/random_explore.cc.o] Error 1
make[2]: *** Waiting for unfinished jobs....
make[1]: *** [CMakeFiles/Makefile2:784: CMakeFiles/rm_decision_test_node.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
cd /home/dan/learn/MA_Sentry/build/rm_decision; catkin build --get-env rm_decision | catkin env -si  /usr/bin/make --jobserver-auth=3,4; cd -

...................................
