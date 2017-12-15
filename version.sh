#!/bin/sh
<<<<<<< HEAD
#SNV_VERSION=`git show -s --format=%h`
echo -n "4.0.1"
=======
BASE_VERSION="4.0.1"
if [ -d .git ]; then
	GIT_COMMIT=`git show -s --format=%h`
	printf "$BASE_VERSION+git$GIT_COMMIT"
else
	printf "$BASE_VERSION+gitUNKNOWN"
fi
>>>>>>> 4.0_Foscam_Autotracking
