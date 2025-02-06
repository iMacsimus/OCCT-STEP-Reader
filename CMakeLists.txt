cmake_minimum_required(VERSION 3.13)

project(OCCT_Parser)
set(CMAKE_CXX_STANDARD 20)

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

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} occd_lib)

add_executable(NURBSS2STEP src/tostep.cpp)

