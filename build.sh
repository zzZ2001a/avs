# CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
# echo -e "\033[0;32mBuild: ${CURDIR} \033[0m";
# CMAKE=""; 
# if [ "$( cmake  --version 2>&1 | grep version | awk '{print $3 }' | awk -F '.' '{print $1}' )" == 3 ] ; then CMAKE=cmake; fi
# if [ "$( cmake3 --version 2>&1 | grep version | awk '{print $3 }' | awk -F '.' '{print $1}' )" == 3 ] ; then CMAKE=cmake3; fi
# if [ "$CMAKE" == "" ] ; then echo "Can't find cmake > 3.0"; exit; fi


# MODE=Debug
# TARGETS=()
# CMAKE_FLAGS=()
# OUTPUT=build
# CMAKE_FLAGS+=( "-DCMAKE_BUILD_TYPE=$MODE" ) 
# CMAKE_FLAGS+=( "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" )
# CMAKE_FLAGS+=( "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=${CURDIR}/${OUTPUT}/${MODE}/bin" )
# CMAKE_FLAGS+=( "-DCMAKE_ARCHIVE_OUTPUT_DIRECTORY=${CURDIR}/${OUTPUT}/${MODE}/lib" )
# CMAKE_FLAGS+=( "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=${CURDIR}/${OUTPUT}/${MODE}/lib" )

# echo "$CMAKE_FLAGS"
# if ! ${CMAKE} -H${CURDIR} -B"${CURDIR}/${OUTPUT}/${MODE}" "${CMAKE_FLAGS[@]}";
# then
#   echo -e "\033[1;31mfailed \033[0m"
#   exit 1;
# fi 
# echo -e "\033[0;32mdone \033[0m";
mkdir build
cd build
cmake ..
# cmake -G "Visual Studio 16 2019" -T v142 ..
make



