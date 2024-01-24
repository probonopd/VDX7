#/usr/bin/bash

# By default, installs to user's local directories
# Change these as appropriate
INSTALLDIR=~/.local/bin
CONFIGDIR=~/.config
INSTALLDIR_LV2=~/.lv2


# Copy files

APP=vdx7

# App
mkdir -p ${INSTALLDIR}
cp ${APP} ${INSTALLDIR}

# config file
mkdir -p ${CONFIGDIR}/${APP}
cp ${APP}.ram ${CONFIGDIR}/${APP}

# LV2
if test -d ${APP}.lv2 ; then
	mkdir -p ${INSTALLDIR_LV2}
	cp -r ${APP}.lv2 ${INSTALLDIR_LV2}
fi
