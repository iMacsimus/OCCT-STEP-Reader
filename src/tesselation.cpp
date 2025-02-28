#include <filesystem>
#include "common.hpp"

void tesselate_shape(const TopoDS_Shape& shape, std::filesystem::path path, Standard_Real deflection) {
  Handle(TDocStd_Application) app = new TDocStd_Application;
  Handle(TDocStd_Document) doc;
  app->NewDocument("ConvSTP2obj", doc);
  
  TopLoc_Location aLoc;
  BRepMesh_IncrementalMesh tesselator(shape, deflection, true, 0.5, true);
  for (TopExp_Explorer aFaceIter (shape, TopAbs_FACE); aFaceIter.More(); aFaceIter.Next()) {
    const TopoDS_Face& aFace = TopoDS::Face(aFaceIter.Current());
    Handle(Poly_Triangulation) aT = BRep_Tool::Triangulation (aFace, aLoc);
    BRepLib_ToolTriangulatedShape::ComputeNormals(aFace, aT);
  }

  Handle(XCAFDoc_ShapeTool) newShapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
  newShapeTool->AddComponent(doc->Main(), shape);

  RWObj_CafWriter writer(path.c_str());
  TColStd_IndexedDataMapOfStringString map;

  MyProgressIndicator indicator;
  auto range = indicator.Start();
  
  auto name = path.filename();
  name.replace_extension("");

  std::cout << std::string("Tesselating " + name.string() + "... ") << std::flush;
  writer.Perform(doc, map, range);
  std::cout << "Done." << std::endl;
}