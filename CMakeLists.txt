cmake_minimum_required(VERSION 3.13)

project(OCCT_Parser)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DOCC_CONVERT_SIGNALS)

set(OCCT_LIBS 
  TKBinL
  TKBin
  TKBinTObj
  TKBinXCAF
  TKBool
  TKBO
  TKBRep
  TKCAF
  TKCDF
  TKDECascade
  TKDEGLTF
  TKDEIGES
  TKDEOBJ
  TKDEPLY
  TKDE
  TKDESTEP
  TKDESTL
  TKDEVRML
  TKernel
  TKG2d
  TKG3d
  TKGeomAlgo
  TKGeomBase
  TKHLR
  TKLCAF
  TKMath
  TKMesh
  TKPrim
  TKRWMesh
  TKService
  TKShHealing
  TKStdL
  TKStd
  TKTObj
  TKTopAlgo
  TKV3d
  TKVCAF
  TKXCAF
  TKXmlL
  TKXml
  TKXmlTObj
  TKXmlXCAF
  TKXSBase
)
link_directories(external/OCCT/linux/lib)

add_library(occd_lib STATIC src/dummy.cpp)
target_link_libraries(occd_lib ${OCCT_LIBS})
target_include_directories(occd_lib PUBLIC ${PROJECT_SOURCE_DIR}/external/OCCT/linux/include/)

add_executable(
  ${PROJECT_NAME} 
    src/main.cpp 
    src/tesselation.cpp
    src/convertion2nurbs.cpp
    src/command_line_arguments.cpp)
target_link_libraries(${PROJECT_NAME} occd_lib)

