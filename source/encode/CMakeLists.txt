message("*****************source/encode****************")
message(${CMAKE_CURRENT_SOURCE_DIR})

#获取当前路径下源文件
file(GLOB SRC *.h *.cpp *.c)

#添加头文件路径
include_directories(${ROOT_PATH}/dependencies/draco/src ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/../../../ ${ROOT_PATH}/dependencies/stb ${ROOT_PATH}/dependencies/hpm-HPM-15.0/inc ${ROOT_PATH}/dependencies/hpm-HPM-15.0/src ${ROOT_PATH}/dependencies/hpm-HPM-15.0/app)

#生成可执行文件
add_executable(encoder ${SRC})

#Draco,hpm库路径
set(DRACO_LIB_PATH $<TARGET_FILE:draco>)
find_library(COMMON_LIB_PATH NAMES CommonLib)
find_library(ENCODER_LIB_PATH NAMES EncoderLib)
link_directories(${DRACO_LIB_PATH} ${COMMON_LIB_PATH} ${ENCODER_LIB_PATH})

#链接Draco,hpm库
target_link_libraries(encoder PUBLIC ${DRACO_LIB} CommonLib EncoderLib)
#message(${DRACO_LIB})
set_target_properties( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)



