cmake_minimum_required( VERSION 2.8 )

project( CamLidarCal )

# opencv relative
set( CMAKE_CXX_STANDARD 14 )
find_package( OpenCV REQUIRED )

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
vtk_module_config(VTK
  vtkCommonCore
  vtkFiltersSources
  vtkFiltersSources
  vtkInteractionStyle
  vtkRendering${VTK_RENDERING_BACKEND}
)
include(${VTK_USE_FILE})

include_directories("/usr/include/eigen3")
include_directories("./include/")

add_executable( solution ./src/main.cpp ./src/pickArea.cpp  ./src/lidar.cpp ./src/utils.cpp ./src/cam.cpp)
# add_executable( solution ./src/areaPicking.cpp )
target_link_libraries( solution ${OpenCV_LIBS} ${PCL_LIBRARIES} ${VTK_LIBRARIES} )