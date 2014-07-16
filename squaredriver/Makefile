##############################################################################
#
#    file                 : Makefile
#    created              : Tue Jul 15 19:49:24 IST 2014
#    copyright            : (C) 2002 Feroze Naina
#
##############################################################################

ROBOT       = squaredriver
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp driver.cpp opponent.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml acura-nsx-sz.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-squaredriver_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-squaredriver_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
