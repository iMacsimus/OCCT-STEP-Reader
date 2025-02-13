#include <thread>
#include <ranges>
#include <format>
using std::ranges::views::iota;


#include "common.hpp"

void output_nurbs(Geom_BSplineSurface *bspline, std::ostream &fout) {
  if (bspline->IsUPeriodic()) {
    bspline->SetUNotPeriodic();
  }
  if (bspline->IsVPeriodic()) {
    bspline->SetVNotPeriodic();
  }
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

void output_rbezier(Geom_BezierSurface *bezier, std::ostream &fout)
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

void convert2nurbs(
      int shape_id,
      TopoDS_Shape shape, 
      std::optional<std::filesystem::path> nurbs_out,
      std::optional<Statistics> &stats,
      std::optional<TopoDS_Shape> &conv_shape,
      std::optional<TopoDS_Shape> &conv_shape_notrim) {
  std::cout << "Divide Closed...";
  ShapeUpgrade_ShapeDivideClosed divider(shape);
  divider.Perform();
  shape = divider.Result();
  println("Done.");

  std::optional<std::ofstream> fout = (nurbs_out) 
                                      ? std::ofstream(nurbs_out.value()) 
                                      : std::optional<std::ofstream>{};

  int count = 0;
  std::vector<std::string> fails;
  std::vector<std::pair<std::string, TopoDS_Face>> failed_faces;
  std::array<int, geom_abs2str.size()> face_type_counts = {};

  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    TopoDS_Face face = TopoDS::Face(ex.Current());
    BRepAdaptor_Surface surface(face);
    auto type = surface.GetType();
    ++face_type_counts[type];

    print("Output {} face({})...", count, geom_abs2str[type]);
    ++count;
    if (type == GeomAbs_BSplineSurface) {
      auto bspline_handler = surface.BSpline();
      auto bspline = bspline_handler.get();
      if (nurbs_out) {
        output_nurbs(bspline, fout.value());
      }
      if (conv_shape) {
        //TODO
      }
      if (conv_shape_notrim) {
        //TODO
      }
    } else if (type == GeomAbs_BezierSurface) {
      auto bezier_handler = surface.Bezier();
      auto bezier = bezier_handler.get();
      if (nurbs_out) {
        output_rbezier(bezier, fout.value());
      }
      if (conv_shape) {
        //TODO
      }
      if (conv_shape_notrim) {
        //TODO
      }
    } else {
      print("Converting to Bspline...");
      try {
        OCC_CATCH_SIGNALS

        BRepBuilderAPI_NurbsConvert convertor(face);
        face = TopoDS::Face(convertor.Shape());
        surface = face;
        auto bspline_handler = surface.BSpline();
        auto bspline = bspline_handler.get();
        if (nurbs_out) {
          output_nurbs(bspline, fout.value());
        }
        if (conv_shape) {
          //TODO
        }
        if (conv_shape_notrim) {
          //TODO
        }
      } catch(Standard_Failure &theExec) {
        std::cout << "Failed. Skip." << std::endl;
        auto message  = std::to_string(count)
                      + "(" + geom_abs2str[type] + "): "
                      + theExec.GetMessageString();
        std::cerr << "\t" << message << std::endl;
        fails.push_back(message);
        std::string aName = std::format("face_{}_{}.brep", shape_id, count-1);
        failed_faces.push_back({ aName, face });
        continue;
      }
    }
    println("Done.");
  }

  if (stats) {
    auto &stats_ref = stats.value();
    stats_ref.faces_count += count;
    for (auto i: rv::iota(0ull, stats_ref.face_type_counts.size())) {
      stats_ref.face_type_counts[i] += face_type_counts[i];
    }
    std::copy(fails.begin(), fails.end(), std::back_inserter(stats_ref.fails));
    std::copy(failed_faces.begin(), failed_faces.end(), std::back_inserter(stats_ref.failed_faces));
  }
}