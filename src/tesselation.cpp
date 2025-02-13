#include <filesystem>
#include "occt_headers.hpp"

void export_to_stl(const TopoDS_Shape& shape, std::filesystem::path path) {
  StlAPI_Writer writer;
  writer.Write(shape, path.c_str());
}

void tesselate_solid(const TopoDS_Solid& shape, std::filesystem::path save_path) {
    // Тесселяция с Deflection = 0.1
    double deflection = 0.1;
    BRepMesh_IncrementalMesh tesselator(shape, deflection, true, 0.5f, true);
    export_to_stl(shape, save_path);
}