#include <thread>
#include <ranges>
#include <format>
using std::ranges::views::iota;


#include "occt_headers.hpp"

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

class ShapeWriter
{
public:
  ShapeWriter() = default;
public:
  void process(TopoDS_Shape shape, std::ostream &fout);
  int faces_count() const { return count_; }
  int faces_count(int type_id) const { return type_counts_[type_id]; }
  const std::vector<std::string> &fails() const { return fails_; }
private:
  int count_ = 0;
  std::array<int, type_names.size()> type_counts_ = {};
  std::vector<std::string> fails_ = {};
};

void ShapeWriter::process(TopoDS_Shape shape, std::ostream &fout) {
  ShapeUpgrade_ShapeDivideClosed divider(shape);
  divider.Perform();
  shape = divider.Result();

  TopExp_Explorer ex;
  for (ex.Init(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    std::cout << "Output " << count_++ << " surface(Type: ";
    TopoDS_Face face = TopoDS::Face(ex.Current());
    BRepAdaptor_Surface surface(face);
    auto type = surface.GetType();
    std::cout << type_names[type] << ")..." << std::flush;
    ++type_counts_[type];
    
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
        auto message  = std::to_string(count_)
                      + "(" + type_names[type] + "): "
                      + theExec.GetMessageString();
        std::cerr << "\t" << message << std::endl;
        fails_.push_back(message);
        std::string aName = std::string("face") + std::to_string(count_) + std::string(".brep");
        BRepTools::Write(face, aName.c_str());
        continue;
      }
      
    }
    std::cout << "Done." << std::endl;
    std::string aName = std::string("face") + std::to_string(count_) + std::string(".brep");
    BRepTools::Write(face, aName.c_str());
    getchar();
  }
}

void write_stats(
    const ShapeWriter &writer,
    std::ostream &out) {
  out << "Stats:" << std::endl;
  for (auto id: iota(0ull, type_names.size())) {
    out << type_names[id] << " " << writer.faces_count(id) << std::endl;
  }
}

void write_fails(
    const ShapeWriter &writer,
    std::ostream &out) {
  out << "Fails: " << writer.fails().size() << std::endl;
  for (auto &fail: writer.fails()) {
    out << fail << std::endl;
  }
  out << "All failed faces were dumbed to .brep files" << std::endl;
}