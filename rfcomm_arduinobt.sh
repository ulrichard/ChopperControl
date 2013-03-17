#! /bin/sh
# further reading : 
# http://superuser.com/questions/374513/bluetooth-pairing-on-the-command-line-in-ubuntu-11-10
# http://www.dankrill.com/lab/index.php?content=procedures&set=b&item=11

sudo rfcomm connect 0 00:07:80:90:8B:EC &
#sudo minicom -D /dev/rfcomm0 -b 9600


