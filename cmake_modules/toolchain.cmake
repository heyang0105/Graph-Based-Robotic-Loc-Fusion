#toolchain cmake file 
#cmake -DCMAKE_TOOLCHAIN_FILE=~/toolchain.cmake ../

SET(CMAKE_SYSTEM_NAME Linux)

SET(TOOLCHAIN_DIR "/usr/local/arm")

#specify the cross compiler
SET(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}/bin/arm-linux-gnueabihf-gcc CACHE FILEPATH "Archiver")
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}/bin/arm-linux-gnueabihf-g++ CACHE FILEPATH "Archiver")
#SET(CMAKE_GFORTRAN  ${TOOLCHAIN_DIR}/bin/arm-linux-gnueabihf-gfortran)
SET(CMAKE_AR ${TOOLCHAIN_DIR}/bin/arm-linux-gnueabihf-ar CACHE FILEPATH "Archiver")
SET(CMAKE_AS ${TOOLCHAIN_DIR}/bin/arm-linux-gnueabihf-as CACHE FILEPATH "Archiver")
SET(CMAKE_LD  ${TOOLCHAIN_DIR}/bin/arm-linux-gnueabihf-ld CACHE FILEPATH "Archiver")
SET(CMAKE_NM ${TOOLCHAIN_DIR}/bin/arm-linux-gnueabihf-nm CACHE FILEPATH "Archiver")
SET(CMAKE_STRIP  ${TOOLCHAIN_DIR}/bin/arm-linux-gnueabihf-strip CACHE FILEPATH "Archiver")

# where is the target environment 
#SET(CMAKE_FIND_ROOT_PATH  ${TOOLCHAIN_DIR} ${3RDPART_LIBS_DIR})
SET(CMAKE_FIND_ROOT_PATH  ${TOOLCHAIN_DIR})

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
