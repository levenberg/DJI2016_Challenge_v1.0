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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/yangpc/workspace/DJI2016_Challenge_v1.0/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/yangpc/workspace/DJI2016_Challenge_v1.0/install/lib/python2.7/dist-packages:/home/yangpc/workspace/DJI2016_Challenge_v1.0/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/yangpc/workspace/DJI2016_Challenge_v1.0/build" \
    "/usr/bin/python" \
    "/home/yangpc/workspace/DJI2016_Challenge_v1.0/src/dji_sdk/setup.py" \
    build --build-base "/home/yangpc/workspace/DJI2016_Challenge_v1.0/build/dji_sdk" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/yangpc/workspace/DJI2016_Challenge_v1.0/install" --install-scripts="/home/yangpc/workspace/DJI2016_Challenge_v1.0/install/bin"
