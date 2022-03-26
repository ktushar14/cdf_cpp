cmake_minimum_required(VERSION 3.10)

project(coverage-search LANGUAGES CXX)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release ... FORCE)
ENDIF()

set(CMAKE_CXX_FLAGS "-Werror") # Treat all warnings as errors

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)

find_package(smpl REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -g")

# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=thread -g -O1 -fPIE -pie")
# set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=thread -g -O1 -fPIE -pie")

# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-omit-frame-pointer -fsanitize=thread -g -O1 -fPIE -pie")
# set(CMAKE_LINKER_FLAGS_RELEASE "${CMAKE_LINKER_FLAGS_RELEASE} -fno-omit-frame-pointer -fsanitize=thread -g -O1 -fPIE -pie")

#############
# LIBRARIES #
#############

add_library(covsearch STATIC
    src/robot_space.cpp
    src/xyenv.cpp
    src/xytheta.cpp
    src/movingai.cpp
    src/planner.cpp
    src/wastar.cpp
    src/constants.cpp
    src/types.cpp
)

target_link_libraries(covsearch smpl::smpl Qt5::Core Qt5::Widgets Threads::Threads -llog4cxx)
set_property(TARGET covsearch PROPERTY CXX_STANDARD 17)
target_include_directories(covsearch PUBLIC include)

add_library(visualize STATIC
    src/dialog.ui
    src/dialog.cpp
    src/map_item.cpp
    src/robot_item.cpp
)
target_link_libraries(visualize Qt5::Core Qt5::Widgets -llog4cxx)
set_property(TARGET visualize PROPERTY CXX_STANDARD 17)
target_include_directories(visualize PUBLIC include)

###############
# EXECUTABLES #
###############

# Apparently if headers were in the same directory as the source files, you
# wouldn't need to add "include/qt/dialog.h" as an argument to add_executable()
# See answer to: https://github.com/bincrafters/community/issues/466
# and AUTOMOC documentation at: https://cmake.org/cmake/help/latest/prop_tgt/AUTOMOC.html
add_executable(persistent tests/persistent.cpp include/qt/dialog.h)
target_link_libraries(persistent PUBLIC visualize covsearch smpl::smpl)
set_property(TARGET persistent PROPERTY CXX_STANDARD 17)

add_executable(persistent_nosim tests/persistent_nosim.cpp)
target_link_libraries(persistent_nosim PUBLIC covsearch smpl::smpl)
set_property(TARGET persistent_nosim PROPERTY CXX_STANDARD 17)

add_executable(dubins_macro_generator tests/dubins_macro_generator.cpp)
target_link_libraries(dubins_macro_generator PUBLIC smpl::smpl)
set_property(TARGET dubins_macro_generator PROPERTY CXX_STANDARD 17)

add_executable(gen_pattern_no_sampling tests/gen_pattern_no_sampling.cpp)
target_link_libraries(gen_pattern_no_sampling PUBLIC smpl::smpl)
set_property(TARGET gen_pattern_no_sampling PROPERTY CXX_STANDARD 17)

add_executable(smpl_dubins_test tests/smpl_dubins_test.cpp)
target_link_libraries(smpl_dubins_test PUBLIC smpl::smpl)
set_property(TARGET smpl_dubins_test PROPERTY CXX_STANDARD 17)

# Apparently if headers were in the same directory as the source files, you
# wouldn't need to add "include/qt/dialog.h" as an argument to add_executable()
# See answer to: https://github.com/bincrafters/community/issues/466
# and AUTOMOC documentation at: https://cmake.org/cmake/help/latest/prop_tgt/AUTOMOC.html
add_executable(visualize_macro_actions tests/visualize_macro_actions.cpp include/qt/dialog.h)
target_link_libraries(visualize_macro_actions PUBLIC visualize covsearch smpl::smpl)
set_property(TARGET visualize_macro_actions PROPERTY CXX_STANDARD 17)
