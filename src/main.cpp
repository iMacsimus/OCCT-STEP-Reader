#include <iostream>
#include <cassert>
#include <fstream>
#include <format>
#include <array>

#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <Poly_Triangulation.hxx>
#include <GeomConvert.hxx>
#include <Geom_Surface.hxx>
#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <OSD.hxx>

constexpr std::array type_names = {
  "GeomAbs_Plane",
  "GeomAbs_Cylinder",
  "GeomAbs_Cone",
  "GeomAbs_Sphere",
  "GeomAbs_Torus",
  "GeomAbs_BezierSurface",
  "GeomAbs_BSplineSurface",
  "GeomAbs_SurfaceOfRevolution",
  "GeomAbs_SurfaceOfExtrusion",
  "GeomAbs_OffsetSurface",
  "GeomAbs_OtherSurface"
};

void output_nurbs(Geom_BSplineSurface *bspline, std::ofstream &fout) {
  fout << std::format("n = {}\nm = {}\n", bspline->NbUPoles()-1, bspline->NbVPoles()-1);
  fout << "points:" << std::endl;
  for (auto &point: bspline->Poles()) {
    fout << std::format("{{{}, {}, {}}} ", point.X(), point.Y(), point.Z());
  }
  fout << std::endl;
  fout << "weights:" << std::endl;
  if (bspline->Weights() != nullptr) {
    for (auto &weight: *bspline->Weights()) {
      fout << weight << " ";
    }
  } else {
    for (int i = 0; i < bspline->NbUPoles(); ++i) 
    for (int j = 0; j < bspline->NbVPoles(); ++j)
    {
      fout << 1.0f << " ";
    }
  }
  
  fout << std::endl;
  fout << std::format("u_degree: {}\nv_degree: {}\n", bspline->UDegree(), bspline->VDegree());
  fout << "u_knots: ";
  for (auto &knot: bspline->UKnotSequence()) {
    fout << knot << " ";
  }
  fout << std::endl;
  fout << "v_knots: ";
  for (auto &knot: bspline->VKnotSequence()) {
    fout << knot << " ";
  }
  fout << std::endl;
}

void output_rbezier(Geom_BezierSurface *bezier, std::ofstream &fout)
{
  fout << std::format("n = {}\nm = {}\n", bezier->NbUPoles()-1, bezier->NbVPoles()-1);
  fout << "points:" << std::endl;
  for (auto &point: bezier->Poles()) {
    fout << std::format("{{{}, {}, {}}} ", point.X(), point.Y(), point.Z());
  }
  fout << std::endl;
  fout << "weights:" << std::endl;
  if (bezier->Weights() != nullptr) {
    for (auto &weight: *bezier->Weights()) {
      fout << weight << " ";
    }
  } else {
    for (int i = 0; i < bezier->NbUPoles(); ++i) 
    for (int j = 0; j < bezier->NbVPoles(); ++j)
    {
      fout << 1.0f << " ";
    }
  }
  fout << std::endl;
  fout << std::format("u_degree: {}\nv_degree: {}\n", bezier->UDegree(), bezier->VDegree());
  fout << "u_knots: ";
  Standard_Real umin, umax, vmin, vmax;
  bezier->Bounds(umin, umax, vmin, vmax);
  for (int i = 0; i < (bezier->NbUPoles()+bezier->UDegree()+1); ++i) {
    fout << (i < (bezier->NbUPoles()+bezier->UDegree()+1)/2 ? umin : umax) << " ";
  }
  fout << std::endl;
  fout << "v_knots: ";
  for (int i = 0; i < (bezier->NbUPoles()+bezier->UDegree()+1); ++i) {
    fout << (i < (bezier->NbUPoles()+bezier->UDegree()+1)/2 ? vmin : vmax) << " ";
  }
  fout << std::endl;
}

void transfer_all_shapes(STEPControl_Reader &reader, std::ofstream &fout) {
  std::cout << "Transferring roots..." << std::flush;
  reader.TransferRoots();
  std::cout << "Done." << std::endl;

  auto shapes_for_transfer = reader.NbShapes();
  std::array<int, type_names.size()> type_counts = {};
  int count = 0;
  std::vector<std::string> fails = {};
  for (int i = 1; i <= shapes_for_transfer; ++i) {
    std::cout << std::format("Transferring shape {}/{}...", i, shapes_for_transfer) << std::endl;
    auto shape = reader.Shape(i);
    TopExp_Explorer ex;
    for (ex.Init(shape, TopAbs_FACE); ex.More(); ex.Next()) {
      std::cout << "Output " << count++ << " surface(Type: ";
      TopoDS_Face face = TopoDS::Face(ex.Current());
      BRepAdaptor_Surface surface(face);
      auto type = surface.GetType();
      std::cout << type_names[type] << ")..." << std::flush;
      ++type_counts[type];
      
      if (type == GeomAbs_BSplineSurface) {
        auto bspline_handler = surface.BSpline();
        auto bspline = bspline_handler.get();
        output_nurbs(bspline, fout);
      } else if (type == GeomAbs_BezierSurface) {
        auto bezier_handler = surface.Bezier();
        auto bezier = bezier_handler.get();
        output_rbezier(bezier, fout);
      } else {
        std::cout << "Converting to Bspline..." << std::flush;
        try {
          OCC_CATCH_SIGNALS

          BRepBuilderAPI_NurbsConvert convertor(face);
          face = TopoDS::Face(convertor.Shape());
          surface = face;
          auto bspline_handler = surface.BSpline();
          auto bspline = bspline_handler.get();
          output_nurbs(bspline, fout);
        } catch(Standard_Failure &theExec) {
          std::cout << "Failed. Skip." << std::endl;
          std::cerr << "\t" << theExec << std::endl;
          fails.push_back(std::to_string(count)+": "+theExec.GetMessageString());
          std::string aName = std::string("face") + std::to_string(count) + std::string(".brep");
          BRepTools::Write(face, aName.c_str());
          continue;
        }
        
      }
      std::cout << "Done." << std::endl;
    }
  }
  std::cout << "-------------" << std::endl;
  std::cout << "Stats:" << std::endl;
  for (int i = 0; i < type_names.size(); ++i) {
    std::cout << type_names[i] << ": " << type_counts[i] << std::endl;
  }
  std::cout << "-------------" << std::endl;
  std::cout << "Fails: " << fails.size() << std::endl;
  for (auto &fail: fails) {
    std::cout << fail << std::endl;
  }
  std::cout << "All failed faces were dumped to .brep files" << std::endl;
}

int main(int argc, const char **argv) {
  OSD::SetSignal(false);
  STEPControl_Reader reader;
  IFSelect_ReturnStatus stat = reader.ReadFile(argv[1]);
  reader.PrintCheckLoad(true, IFSelect_PrintCount::IFSelect_ListByItem);

  std::ofstream fout(std::string(argv[2])+".nurbss");
  transfer_all_shapes(reader, fout);
  
  return 0;
}