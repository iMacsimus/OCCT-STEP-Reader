#include <iostream>
#include <cassert>
#include <fstream>
#include <format>

#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS.hxx>
#include <BRepTools.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>


int main(int argc, const char **argv) {
  STEPControl_Reader reader;
  IFSelect_ReturnStatus stat = reader.ReadFile(argv[1]);
  reader.PrintCheckLoad(true, IFSelect_PrintCount::IFSelect_ListByItem);

  std::ofstream fout(std::string(argv[2])+".nurbss");
  
  auto roots_for_transfer = reader.NbRootsForTransfer();
  for (int i = 1; i <= roots_for_transfer; ++i) {
    std::cout << "Transferring root " << i << "...";
    reader.TransferOneRoot(i);
    std::cout << "Done." << std::endl;
  }

  auto shapes_for_transfer = reader.NbShapes();
  for (int i = 1; i <= shapes_for_transfer; ++i) {
    std::cout << "Transferring shape " << shapes_for_transfer << "..." << std::endl;
    auto shape = reader.Shape(i);
    // load each solid as an own object
    TopExp_Explorer ex;
    int count = 0;
    for (ex.Init(shape, TopAbs_FACE); ex.More(); ex.Next()) {
      std::cout << "Output " << count++ << " surface(Type: ";
      const TopoDS_Face &face = TopoDS::Face(ex.Current());
      BRepAdaptor_Surface surface(face);
      auto type = surface.GetType();
      std::cout << type << ")...";
      if (type == GeomAbs_BSplineSurface) {
        auto bspline = surface.BSpline().get();
        fout << std::format("n = {}\nm = {}\n", bspline->NbUPoles()-1, bspline->NbVPoles()-1);
        fout << "points:" << std::endl;
        for (auto &point: bspline->Poles()) {
          fout << std::format("{{{}, {}, {}}} ", point.X(), point.Y(), point.Z());
        }
        fout << std::endl;
        fout << "weights:" << std::endl;
        for (auto &weight: *bspline->Weights()) {
          fout << weight << " ";
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
      } else if (type == GeomAbs_BezierSurface) {
        auto bezier = surface.Bezier().get();
        fout << std::format("n = {}\nm = {}\n", bezier->NbUPoles()-1, bezier->NbVPoles()-1);
        fout << "points:" << std::endl;
        for (auto &point: bezier->Poles()) {
          fout << std::format("{{{}, {}, {}}} ", point.X(), point.Y(), point.Z());
        }
        fout << std::endl;
        fout << "weights:" << std::endl;
        for (auto &weight: *bezier->Weights()) {
          fout << weight << " ";
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
      } else if (type == GeomAbs_Plane) {
        //TODO
      }
      std::cout << "Done." << std::endl;
    } 
  }

  
  return 0;
}