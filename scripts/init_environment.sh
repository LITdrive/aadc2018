#!/bin/bash

# get directory of THIS file
SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# get directory of the PROJECT (one folder up)
export PROJECT_ROOT=$(readlink -f ${SCRIPTS_DIR}/..)
echo "Fixed ADTF environment for loading a project from ${PROJECT_ROOT}"

# fix Qt backend for ADTF
export QT_QUICK_BACKEND=software

# patch the BIN_DIR environment variable for ADTF
perl -0777 -i -pe 's/<name>BIN_DIR<\/name>\s+<value>.*?<\/value>/<name>BIN_DIR<\/name>\n      <value>$ENV{'PROJECT_ROOT'}\/_install\/linux64\/bin<\/value>/g' ${PROJECT_ROOT}/config/LiveVisualization/adtfsessions/adtfsystems/default_system.adtfsystem
perl -0777 -i -pe 's/<name>BIN_DIR<\/name>\s+<value>.*?<\/value>/<name>BIN_DIR<\/name>\n      <value>$ENV{'PROJECT_ROOT'}\/_install\/linux64\/bin<\/value>/g' ${PROJECT_ROOT}/config/UserConfiguration/adtfsessions/adtfsystems/default_system_linux.adtfgraph

