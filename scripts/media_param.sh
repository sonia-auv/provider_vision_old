#!/bin/bash

help="-h"
#"$1" = "$help"
if [ "$1" = "$help" ] ; then
  echo Usage: media_param.sh [get/set] media feature [value]
elif [ "$1" = "get" ] ; then
  rosservice call /provider_vision/get_media_param_list $2 $3
elif [ "$1" = "set" ] ; then
  rosservice call /provider_vision/set_media_param_list $2 $3 $4
fi
