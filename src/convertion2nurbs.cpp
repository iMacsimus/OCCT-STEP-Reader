#include <thread>
#include <cassert>

#include "common.hpp"

template<typename T>
void binout(std::ostream &fout, const T& value) {
  fout.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

void output_nurbs(Geom_BSplineSurface *bspline, std::ostream &fout) {
  if (bspline->IsUPeriodic()) {
    bspline->SetUNotPeriodic();
  }
  if (bspline->IsVPeriodic()) {
    bspline->SetVNotPeriodic();
  }

  int n = bspline->NbUPoles()-1, m = bspline->NbVPoles()-1;
  binout(fout, n);
  binout(fout, m);

  for (auto &point: bspline->Poles()) {
    float point_values[4] = { 
      static_cast<float>(point.X()), 
      static_cast<float>(point.Y()), 
      static_cast<float>(point.Z()), 
      1.0f 
    };
    binout(fout, point_values);
  }
  
  if (bspline->Weights() != nullptr) {
    for (float weight: *bspline->Weights()) {
      binout(fout, weight);
    }
  } else {
    for (int i = 0; i < (n+1)*(m+1); ++i) {
      float weight = 1.0f;
      binout(fout, weight);
    }
  }

  int u_deg = bspline->UDegree(), v_deg = bspline->VDegree();
  binout(fout, u_deg);
  binout(fout, v_deg);
  for (float knot: bspline->UKnotSequence()) {
    binout(fout, knot);
  }
  for (float knot: bspline->VKnotSequence()) {
    binout(fout, knot);
  }
}

auto convert_shape(TopoDS_Shape &shape, const TCollection_ExtendedString &name) {
  std::vector<TopoDS_Face> faces;
  BRepBuilderAPI_NurbsConvert convertor;
  int total = 0;
  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    std::cout << "Converting \"" << name << "\"'s " << total << "th face to Bspline..." << std::flush;
    auto face = ex.Current();
    try {
      OCC_CATCH_SIGNALS
      convertor.Perform(face);
      face = convertor.Shape();
      BRepAdaptor_Surface surface(TopoDS::Face(face));
      auto type = surface.GetType();
      if (type != GeomAbs_BSplineSurface) {
        throw Standard_Failure("Type after convertion is not bspline surface");
      }
      faces.push_back(TopoDS::Face(face));
    } catch(Standard_Failure &err) {
      std::cout << "Failed. Skip." << std::endl;
      std::cerr << name + ": " << err << std::endl;
      continue;
    } catch(...) {
      std::cout << "Failed. Skip." << std::endl;
      std::cerr << name + ": " << "Unknown" << std::endl;
      continue;
    }
    ++total;
    std::cout <<"Done." << std::endl;
  }

  return faces;
}

void convert2nurbs(
      const TCollection_ExtendedString &name,
      TopoDS_Shape shape,
      std::ofstream &fout) {
  BRep_Builder builder;

  auto faces = convert_shape(shape, name);
  binout(fout, static_cast<int>(faces.size()));

  std::cout << "Output \"" << name << "\" to .nurbs..." << std::flush;
  for (auto &face: faces) {
    BRepAdaptor_Surface surface(face);
    auto type = surface.GetType();
    //assert(type == GeomAbs_BSplineSurface);

    auto bspline_handler = surface.BSpline();
    auto bspline = bspline_handler.get();
    if (fout) {
      output_nurbs(bspline, fout);
    }
  }
  std::cout << "Done." << std::endl;
}