#include <thread>

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

void output_rbezier(Geom_BezierSurface *bezier, std::ostream &fout)
{
  int n = bezier->NbUPoles()-1, m = bezier->NbVPoles()-1;
  binout(fout, n);
  binout(fout, m);
  for (auto &point: bezier->Poles()) {
    float point_values[4] = { 
      static_cast<float>(point.X()), 
      static_cast<float>(point.Y()), 
      static_cast<float>(point.Z()), 
      1.0f 
    };
    binout(fout, point_values);
  }
  if (bezier->Weights() != nullptr) {
    for (float weight: *bezier->Weights()) {
      binout(fout, weight);
    }
  } else {
    for (int i = 0; i < (n+1)*(m+1); ++i) {
      float weight = 1.0f;
      binout(fout, weight);
    }
  }
  int u_deg = bezier->UDegree(), v_deg = bezier->VDegree();
  binout(fout, u_deg);
  binout(fout, v_deg);
  Standard_Real umin, umax, vmin, vmax;
  bezier->Bounds(umin, umax, vmin, vmax);
  for (int i = 0; i < (bezier->NbUPoles()+bezier->UDegree()+1); ++i) {
    auto knot = (i < (bezier->NbUPoles()+bezier->UDegree()+1)/2 ? umin : umax);
    binout(fout, static_cast<float>(knot));
  }
  for (int i = 0; i < (bezier->NbUPoles()+bezier->UDegree()+1); ++i) {
    auto knot = (i < (bezier->NbUPoles()+bezier->UDegree()+1)/2 ? vmin : vmax);
    binout(fout, static_cast<float>(knot));
  }
}

void convert2nurbs(
      int shape_id, int shapes_total,
      TopoDS_Shape shape, 
      std::optional<std::ofstream> &fout,
      std::optional<Statistics> &stats,
      std::optional<TopoDS_Compound> &conv_shape,
      std::optional<TopoDS_Compound> &conv_shape_notrim) {
  std::cout << "Divide Closed Faces...";
  BRep_Builder builder;
  ShapeUpgrade_ShapeDivideClosed divider(shape);
  divider.Perform();
  shape = divider.Result();
  std::cout << "Done." << std::endl;

  int count = 0, total = 0;
  std::vector<std::string> fails;
  std::vector<std::pair<std::string, TopoDS_Face>> failed_faces;
  std::array<int, geom_abs2str.size()> face_type_counts = {};

  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    ++total;
  }    
  if (fout) {
    std::ofstream &out = fout.value();
    binout(out, total);
  }

  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    TopoDS_Face face = TopoDS::Face(ex.Current());
    BRepAdaptor_Surface surface(face);
    auto type = surface.GetType();
    ++face_type_counts[type];

    std::cout << "[" << (shape_id+1)  << "/" << shapes_total << ", " 
              << (count+1)*100.0f/total << "%] Output " << count 
              << " face(" << geom_abs2str[type] << ")..." << std::flush;
    ++count;
    if (type == GeomAbs_BSplineSurface) {
      auto bspline_handler = surface.BSpline();
      auto bspline = bspline_handler.get();
      if (fout) {
        output_nurbs(bspline, fout.value());
      }
      if (conv_shape) {
        builder.Add(conv_shape.value(), face);
      }
      if (conv_shape_notrim) {
        TopoDS_Wire wr;
        builder.MakeWire(wr);
        BRepBuilderAPI_MakeFace facemaker(bspline_handler, wr);
        auto notrim_face = facemaker.Face();
        builder.Add(conv_shape_notrim.value(), notrim_face);
      }
    } else if (type == GeomAbs_BezierSurface) {
      auto bezier_handler = surface.Bezier();
      auto bezier = bezier_handler.get();
      if (fout) {
        output_rbezier(bezier, fout.value());
      }
      if (conv_shape) {
        builder.Add(conv_shape.value(), face);
      }
      if (conv_shape_notrim) {
        TopoDS_Wire wr;
        builder.MakeWire(wr);
        BRepBuilderAPI_MakeFace facemaker(bezier_handler, wr);
        auto notrim_face = facemaker.Face();
        builder.Add(conv_shape_notrim.value(), notrim_face);
      }
    } else {
      std::cout << "Converting to Bspline..." << std::flush;
      try {
        OCC_CATCH_SIGNALS

        BRepBuilderAPI_NurbsConvert convertor(face);
        face = TopoDS::Face(convertor.Shape());
        surface = face;
        auto bspline_handler = surface.BSpline();
        auto bspline = bspline_handler.get();
        if (fout) {
          output_nurbs(bspline, fout.value());
        }
        if (conv_shape) {
          builder.Add(conv_shape.value(), face);
        }
        if (conv_shape_notrim) {
          TopoDS_Wire wr;
          builder.MakeWire(wr);
          BRepBuilderAPI_MakeFace facemaker(bspline_handler, wr);
          auto notrim_face = facemaker.Face();
          builder.Add(conv_shape_notrim.value(), notrim_face);
        }
      } catch(Standard_Failure &theExec) {
        std::cout << "Failed. Skip." << std::endl;
        auto message  = std::to_string(count)
                      + "(" + geom_abs2str[type] + "): "
                      + theExec.GetMessageString();
        std::cerr << "\t" << message << std::endl;
        fails.push_back(message);
        std::string aName = std::string("face_")
                          + std::to_string(shape_id)
                          +"_"
                          +std::to_string(count-1)
                          +".brep";
        failed_faces.push_back({ aName, face });
        continue;
      }
    }
    std::cout << "Done." << std::endl;
  }

  if (stats) {
    auto &stats_ref = stats.value();
    stats_ref.faces_count += count;
    for (int i = 0; i < stats_ref.face_type_counts.size(); ++i) {
      stats_ref.face_type_counts[i] += face_type_counts[i];
    }
    std::copy(fails.begin(), fails.end(), std::back_inserter(stats_ref.fails));
    std::copy(failed_faces.begin(), failed_faces.end(), std::back_inserter(stats_ref.failed_faces));
  }
}