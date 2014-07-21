#!/bin/sh


if [ $# -eq 0 ] ; then
    /bin/echo "Entering build environment at /home/cantor/fuerte_workspace/dronenav"
    . /home/cantor/fuerte_workspace/dronenav/setup.sh
    $SHELL -i
    /bin/echo "Exiting build environment at /home/cantor/fuerte_workspace/dronenav"
else
    . /home/cantor/fuerte_workspace/dronenav/setup.sh
    exec "$@"
fi


