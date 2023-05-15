set(CMAKE_CXX_STANDARD 11)

aux_source_directory(. DIR_LIB_SRCS)

add_library(socket SHARED ${DIR_LIB_SRCS})

message(STATUS "socket lib make success")
