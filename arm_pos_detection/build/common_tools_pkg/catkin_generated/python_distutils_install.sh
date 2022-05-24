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

echo_and_run cd "/home/ubuntu/ros_ws/arm_pos_detection/src/common_tools_pkg"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ubuntu/ros_ws/arm_pos_detection/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ubuntu/ros_ws/arm_pos_detection/install/lib/python3/dist-packages:/home/ubuntu/ros_ws/arm_pos_detection/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ubuntu/ros_ws/arm_pos_detection/build" \
    "/usr/bin/python3" \
    "/home/ubuntu/ros_ws/arm_pos_detection/src/common_tools_pkg/setup.py" \
     \
    build --build-base "/home/ubuntu/ros_ws/arm_pos_detection/build/common_tools_pkg" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ubuntu/ros_ws/arm_pos_detection/install" --install-scripts="/home/ubuntu/ros_ws/arm_pos_detection/install/bin"
