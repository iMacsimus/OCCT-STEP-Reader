#include <iostream>
#include <cassert>
#include <fstream>
#include <format>
#include <array>
#include <chrono>
#include <thread>
#include <filesystem>
#include <ranges>

#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <OSD.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressRange.hxx>
#include <ShapeUpgrade_ShapeDivideClosed.hxx>
#include <BRep_Builder.hxx>

using std::ranges::views::iota;

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

class MyProgressIndicator : public Message_ProgressIndicator {
public:
    MyProgressIndicator() {}
protected:
    virtual void Show(const Message_ProgressScope& theScope, 
                     const Standard_Boolean isForce) override {
    }
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

void progress_bar(Message_ProgressIndicator &indicator, std::string message) {
  while((1.0 - indicator.GetPosition()) > 1e-4) {
    std::cout << std::format("\r{} {:10.5}%", message, indicator.GetPosition()*100) << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "\r" << message << " Done.                 " << std::endl;
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

int main(int argc, const char **argv) {
  OSD::SetSignal(false);

  std::filesystem::path model_path = argv[1];
  std::filesystem::path result_file_path = argv[2];
  result_file_path.replace_extension(".nurbss");

  ShapeWriter writer;
  std::ofstream fout(std::string(argv[2])+".nurbss");

  MyProgressIndicator indicator;
  Message_ProgressRange range = indicator.Start();

  if (model_path.extension() == ".step"
      || model_path.extension() == ".stp") {
    STEPControl_Reader reader;
    IFSelect_ReturnStatus stat = reader.ReadFile(model_path.c_str());
    reader.PrintCheckLoad(true, IFSelect_PrintCount::IFSelect_ListByItem);
    
    std::thread progress_th(progress_bar, std::reference_wrapper(indicator), "Transferring Roots...");
    reader.TransferRoots(range);
    progress_th.join(); 

    auto shapes_for_transfer = reader.NbShapes();
    for (int i = 1; i <= shapes_for_transfer; ++i) {
      std::cout << std::format("Transferring shape {}/{}...", i, shapes_for_transfer) << std::endl;
      auto shape = reader.Shape(i);
      writer.process(std::move(shape), fout);
    }
  } else if (model_path.extension() == ".brep") {
    BRep_Builder builder;
    TopoDS_Shape shape;

    std::thread progress_th(progress_bar, std::reference_wrapper(indicator), "Reading .brep file...");
    BRepTools::Read(shape, model_path.c_str(), builder, range);
    progress_th.join();

    writer.process(shape, fout);
  } else {
    throw std::invalid_argument(
        std::string("Incorrect format (")
      + model_path.extension().string()
      + "). Must be one of { .step, .stp, .brep }");
  }

  std::ofstream fstats("stats.txt"), ffails("fails.txt");
  write_stats(writer, std::cout);
  write_stats(writer, fstats);
  write_fails(writer, std::cout);
  write_fails(writer, ffails);
  return 0;
}