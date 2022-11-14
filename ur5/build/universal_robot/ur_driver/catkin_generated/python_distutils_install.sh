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

echo_and_run cd "/home/daniel/Desktop/AgarreDual/ur5/src/universal_robot/ur_driver"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/daniel/Desktop/AgarreDual/ur5/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/daniel/Desktop/AgarreDual/ur5/install/lib/python3/dist-packages:/home/daniel/Desktop/AgarreDual/ur5/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/daniel/Desktop/AgarreDual/ur5/build" \
    "/usr/bin/python3" \
    "/home/daniel/Desktop/AgarreDual/ur5/src/universal_robot/ur_driver/setup.py" \
     \
    build --build-base "/home/daniel/Desktop/AgarreDual/ur5/build/universal_robot/ur_driver" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/daniel/Desktop/AgarreDual/ur5/install" --install-scripts="/home/daniel/Desktop/AgarreDual/ur5/install/bin"
