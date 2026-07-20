#!/bin/bash

NATIVE="$VIRTUAL_ENV/lib/python3.11/site-packages/native"

export LD_LIBRARY_PATH="$NATIVE/ntcore/lib:$NATIVE/wpinet/lib:$NATIVE/wpiutil/lib:$NATIVE/wpihal/lib:$NATIVE/wpilib/lib:$NATIVE/wpimath/lib"

./ntclient
