#!/bin/sh



if [ $# -eq 0 ] ; then
    /bin/echo "Entering environment at /usr/local"
    . /usr/local/setup.sh
    $SHELL
    /bin/echo "Exiting build environment at /usr/local"
else
    . /usr/local/setup.sh
    exec "$@"
fi



