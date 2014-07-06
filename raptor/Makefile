##############################################################################
#
#    file                 : Makefile
#    created              : Sun Jul 6 12:03:06 IST 2014
#    copyright            : (C) 2002 Feroze Naina
#
##############################################################################

ROBOT       = raptor
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml buggy.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-raptor_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-raptor_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
