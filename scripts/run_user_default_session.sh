#!/bin/bash

source $(dirname $0)/init_environment.sh

# start configuration editor
cd /opt/ADTF/3.3.3/bin/
/opt/ADTF/3.3.3/bin/adtf_launcher -session=${PROJECT_ROOT}/config/UserConfiguration/adtfsessions/default_session_linux.adtfsession -run
