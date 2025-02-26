#!/bin/bash
set -e

function help_and_exit {
    echo "Cross-Compiler for Linux targets"
#    echo " !!! Don't forget to mount input/output directory and /usr filesystem !!! See README.md"
    echo " !!! This usage info only applies to the build entrypoint, not starting the docker !!!"
    echo
    echo "Build entrypoint arguments: "
    echo "<docker run command> [--wd=<PATH>] [--clean-build] [--debug] [--deploy=<REMOTE_PATH>] [--processes=N]"
    echo "  --wd=<PATH>             --  working directory (where the CMakeLists.txt of the package to be built is, relative to ./)"
#    echo "  --sync                  --  synchronize the targets /usr folder with the docker volume. Do this on first usage, or if libraries/header of the target change."
    echo "  --clean-build           --  deletes the build-<arch>-<os>-<abi> folder from the local tree before compilation"
    echo "  --debug                 --  sets -DCMAKE_BUILD_TYPE=Debug"
    echo "  --deploy=<REMOTE_PATH>  --  use rsync to push the build to <REMOTE_PATH>/<CMAKE_PROJECT_NAME>/build-<arch>-<os>-<abi> on the target"
    echo "  --processes=<N>         --  number of worker threads. Equivalent to -j N. Default 7"
    echo
    exit 1;
}

# handle arguments
#deploy=$(echo "$@" | awk -F= '{a[$1]=$2} END {print(a["--deploy"])}')
# key-value pairs (HOW IS THERE NOT A BUILT IN FOR THIS?)
processes=7
while test $# != 0
do
    left=$(echo "$1" | cut -d "=" -f 1)
    right=$(echo "$1" | cut -d "=" -f 2)
    case "$left" in
    --wd) WD=$right ;;
#    --sync) sync=t ;;
    --clean-build) clean_build=t ;;
    --debug) debug=t ;;
    --deploy) deploy=$right ;;
    --processes) processes=$right ;;
    --help) help_and_exit ;;
    -h) help_and_exit ;;
    esac
    shift
done


# check for mounts
#if ! mountpoint -q $SYSROOT
#then
#    echo "Rootfs not mounted. Run container with -v [local_fs|live_fs]:$SYSROOT"
#    exit 1;
#fi

if ! mountpoint -q $PACKAGE
then
    echo "Package not mounted. Run container with --mount type=bind,src=/path/to/package/root/,dest=$PACKAGE"
    exit 1;
fi

PROJECT=$PACKAGE/$WD

# check for cmake project define
if [ ! -z $deploy ]
then
    CMAKE_PROJECT=$(sed -n -e 's/^project(\(.*\))/\1/p' < $PROJECT/CMakeLists.txt)
    if [[ -z $CMAKE_PROJECT ]] || [[ $(wc -l <<< $CMAKE_PROJECT) -gt 1 ]]
    then
        echo "CMakeLists.txt must contain a single project statement (to set the deploy directory)"
        exit 1;
    fi
fi


## update rootfs environment in the volume
# rsync options:
# verbose, recursive through directories, keep symlink, Relative,
# delete missing, discard links outside of transfered directory
#if [ "$sync" == t ]
#then
#    # reason for --copy-unsafe-links: 
#    # rsync has no --convert-unsafe-absolute-to-relative option or something
#    # so we're gonna have to hard-copy those. But a quick investigation shows
#    # that there arent so many (probably actually bugs in the aarch64 gcc
#    # library), so this should be fine
#
#    # From the manpage or rsync:
#    # --links --copy-unsafe-links
#    #       Turn all unsafe symlinks into files and create all safe symlinks.
#
#    # reason for --ignore-errors:
#    # When backing up a live system, some changes can happen causing IO errors
#    # This is not bad, but rsync then skips file deletion, which shouldnt 
#    # happen, so ignore the errors
#    echo -n "Rsync-ing rootfs... "
#    rsync -tr --delete-after --links --copy-unsafe-links --ignore-errors \
#        --rsh "/usr/bin/sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -l $REMOTE_USER" \
#        --rsync-path="sudo rsync" --timeout=3 \
#        $REMOTE_USER@$REMOTE_IP:/usr $SYSROOT \
#        || true # do not fail on error, for isntance, because pi is down
#
#    # little hacky, but okay:
#    ln -sf $SYSROOT/usr/lib $SYSROOT/lib
#    echo "Rsync done or failed"
#fi

# check for empty rootfs (for instance because rsync failed on first run)
#if [ -z "$(ls -A $SYSROOT)" ]
#then
#    echo "Rootfs is mounted, but empty! Do an initial sync by connecting the Pi."
#    exit 1;
#fi


if [ "$clean_build" == t ]
then
    echo -n "Cleaning build directory... "
    rm -rf $PROJECT/build-$GNU_HOST
    echo "done"
fi

mkdir -p $PROJECT/build-$GNU_HOST
cd $PROJECT/build-$GNU_HOST


# handle build options, then build
CMAKE_EXTRA=
if [ "$debug" == t ]
then
    CMAKE_EXTRA+=" -DCMAKE_BUILD_TYPE=Debug"
fi

cmake -DCMAKE_TOOLCHAIN_FILE=$CROSS_TOOLCHAIN $CMAKE_EXTRA ..
#cmake --build . --parallel 4 # looks cleaner, but somehow broken
make -j $processes # TODO: make the thread count a parameter that can be passed to docker

# TODO: error handling, so we dont upload failed builds

# upload if necessary
if [ ! -z $deploy ]
then
    echo -n "Deploying build... "
    cd $PROJECT && \
    echo build-$GNU_HOST $REMOTE_USER@$REMOTE_IP:"$deploy/$CMAKE_PROJECT/"
    rsync -rR --delete-after --links --copy-unsafe-links --perms \
        --rsh "/usr/bin/sshpass -p $REMOTE_PASSWORD ssh -o StrictHostKeyChecking=no -l $REMOTE_USER" \
        --timeout=3 \
        build-$GNU_HOST $REMOTE_USER@$REMOTE_IP:"$deploy/$CMAKE_PROJECT/"
    #--rsync-path="sudo rsync" \
    echo "build deployed"
else
    echo "Not deploying."
fi

echo "finished"
echo $deploy
exit 0
