##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(cognition_project)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS qt_build roscpp tf)

include_directories(${catkin_INCLUDE_DIRS})
 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/cognition_project/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(cognition_project ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(cognition_project ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS cognition_project RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
