#!/usr/bin/bash

# Change these as appropriate
INSTALLDIR=~/.local
CONFIGDIR=~/.config
INSTALLDIR_LV2=~/.lv2
DESKTOP=~/Desktop
APP=vdx7

rm -f ${INSTALLDIR}/bin/${APP} 
rm -f ${INSTALLDIR}/share/icons/hicolor/128x128/apps/${APP}.png
rm -rf ${CONFIGDIR}/${APP}
rm -rf ${INSTALLDIR_LV2}/${APP}.lv2
rm -rf ${DESKTOP}/${APP}.desktop
