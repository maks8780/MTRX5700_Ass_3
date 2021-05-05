#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/mtrx5700/Documents/University/MTRX5700/MTRX5700_Ass_3/src/SLAM/turtlebot3/turtlebot3_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/mtrx5700/Documents/University/MTRX5700/MTRX5700_Ass_3/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/mtrx5700/Documents/University/MTRX5700/MTRX5700_Ass_3/install/lib/python2.7/dist-packages:/home/mtrx5700/Documents/University/MTRX5700/MTRX5700_Ass_3/build/turtlebot3_teleop/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/mtrx5700/Documents/University/MTRX5700/MTRX5700_Ass_3/build/turtlebot3_teleop" \
    "/usr/bin/python2" \
    "/home/mtrx5700/Documents/University/MTRX5700/MTRX5700_Ass_3/src/SLAM/turtlebot3/turtlebot3_teleop/setup.py" \
     \
    build --build-base "/home/mtrx5700/Documents/University/MTRX5700/MTRX5700_Ass_3/build/turtlebot3_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/mtrx5700/Documents/University/MTRX5700/MTRX5700_Ass_3/install" --install-scripts="/home/mtrx5700/Documents/University/MTRX5700/MTRX5700_Ass_3/install/bin"
