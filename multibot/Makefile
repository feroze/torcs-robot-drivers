##############################################################################
#
#    file                 : Makefile
#    created              : Tue Jul 8 11:04:52 IST 2014
#    copyright            : (C) 2002 Feroze Naina
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = multibot
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp driver.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-multibot_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-multibot_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
