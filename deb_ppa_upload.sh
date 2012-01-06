#!/bin/sh
# build a debian sourcepackage and upload it to the launchpad ppa
export GPGKEY=DA94BB53
export DEBEMAIL="richi@paraeasy.ch"
export DEBFULLNAME="Richard Ulrich"

:${VERSIONNBR:=$(parsechangelog | grep Version | sed -e "s/Version: //g" -e "s/\\~.*//g")}

rm -rf build-debian
rm -rf build
rm -rf debian/choppercontrol
rm -rf debian/tmp

for DISTRIBUTION in precise oneiric natty maverick 
do
	sed -i  -e "s/maverick/${DISTRIBUTION}/g" -e "s/natty/${DISTRIBUTION}/g" -e "s/oneiric/${DISTRIBUTION}/g" -e "s/precise/${DISTRIBUTION}/g" debian/changelog
	dpkg-buildpackage -rfakeroot -S
	dput ppa:richi-paraeasy/ppa ../choppercontrol_${VERSIONNBR}~${DISTRIBUTION}_source.changes
done
