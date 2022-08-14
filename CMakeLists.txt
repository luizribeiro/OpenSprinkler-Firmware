cmake_minimum_required (VERSION 2.8.11)
project (opensprinkler)

file(GLOB SRC src/*.cpp)
add_executable(opensprinkler ${SRC})

target_include_directories (opensprinkler PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(opensprinkler PUBLIC gpiod mosquitto)

install (TARGETS opensprinkler DESTINATION bin)
