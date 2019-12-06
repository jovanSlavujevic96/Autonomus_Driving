# CMake generated Testfile for 
# Source directory: /home/rtrk/myROSworkspace/src/bachelor
# Build directory: /home/rtrk/myROSworkspace/src/bachelor/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_bachelor_roslaunch-check_launch "/home/rtrk/myROSworkspace/src/bachelor/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/rtrk/myROSworkspace/src/bachelor/build/test_results/bachelor/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/rtrk/myROSworkspace/src/bachelor/build/test_results/bachelor" "/opt/ros/melodic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/home/rtrk/myROSworkspace/src/bachelor/build/test_results/bachelor/roslaunch-check_launch.xml' '/home/rtrk/myROSworkspace/src/bachelor/launch' ")
subdirs("gtest")
