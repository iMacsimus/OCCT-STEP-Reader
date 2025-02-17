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

void convert_solid(int shape_id, int shapes_total, TopoDS_Shape &shape) {
  std::string message = std::string("[")
                      + std::to_string(shape_id+1) 
                      + "/" 
                      + std::to_string(shapes_total)
                      + "] Convert to bspline....";
  std::cout << message << std::flush;
  BRepBuilderAPI_NurbsConvert convertor;
  try {
    convertor.Perform(shape);
  } catch(Standard_Failure &err) {
    std::cout << "Failed. Skip." << std::endl;
    std::cerr << std::to_string(shape_id) + ": " << err << std::endl;
    throw;
  }
  shape = convertor.Shape();
}

void convert2nurbs(
      int shape_id, int shapes_total,
      TopoDS_Solid shape, 
      std::optional<std::ofstream> &fout,
      std::optional<Statistics> &stats,
      std::optional<TopoDS_CompSolid> &conv_shape,
      std::optional<TopoDS_CompSolid> &conv_shape_notrim) {
  BRep_Builder builder;

  int count = 0, total = 0;
  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) { ++total; }

  std::vector<std::string> fails;
  std::vector<std::pair<std::string, TopoDS_Solid>> failed_solids;

  try {
    convert_solid(shape_id, shapes_total, shape);
  } catch(Standard_Failure &err) {
    fails.push_back(std::to_string(shape_id)+": "+err.GetMessageString());
    failed_solids.push_back({std::to_string(shape_id)+".brep", shape});
  }

  if (fout) {
    std::ofstream &out = fout.value();
    binout(out, total);
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
      TopoDS_Face face = TopoDS::Face(ex.Current());
      BRepAdaptor_Surface surface(face);
      auto type = surface.GetType();
      assert(type == GeomAbs_BSplineSurface);

      std::cout << "[" << (shape_id+1)  << "/" << shapes_total << ", " 
                << (count+1)*100.0f/total << "%] Output " << count 
                << " face(" << geom_abs2str[type] << ")..." << std::flush;
      ++count;
      auto bspline_handler = surface.BSpline();
      auto bspline = bspline_handler.get();
      if (fout) {
        output_nurbs(bspline, fout.value());
      }
      std::cout << "Done." << std::endl;
    }
  }

  if (conv_shape) {
    builder.Add(conv_shape.value(), shape);
  }

  if (conv_shape_notrim) {
    //TODO
  }

  if (stats) {
    auto &stats_ref = stats.value();
    std::copy(fails.begin(), fails.end(), std::back_inserter(stats_ref.fails));
    std::copy(failed_solids.begin(), failed_solids.end(), std::back_inserter(stats_ref.failed_solids));
  }
}