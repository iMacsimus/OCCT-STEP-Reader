#include <iostream>
#include <cassert>
#include <fstream>
#include <array>
#include <chrono>
#include <thread>
#include <filesystem>
#include <vector>
#include <iostream>
#include <map>
#include <optional>

#include "common.hpp"

void process_solid(
    int shape_id, int shapes_total,
    const TopoDS_Solid &solid,
    std::optional<std::ofstream> &nurbs_out,
    std::optional<std::filesystem::path> stl_out,
    std::optional<Statistics>& stats,
    std::optional<TopoDS_CompSolid>& conv_shape,
    std::optional<TopoDS_CompSolid>& conv_shape_notrim) {
  if (stl_out) {
    std::cout << "[" << (shape_id+1) << "/" << shapes_total, "] Tesselate....";
    auto save_path = stl_out.value();
    tesselate_solid(solid, save_path);
    std::cout << "Done.";
  }
  if (nurbs_out || conv_shape || conv_shape_notrim) {
    convert2nurbs(shape_id, shapes_total, solid, nurbs_out, stats, conv_shape, conv_shape_notrim);
  }
}

void recursive(
    const TDF_Label &label, 
    Handle(XCAFDoc_ColorTool) color_tool, 
    Handle(XCAFDoc_ShapeTool) shape_tool) {
  Quantity_Color color(gp_Vec3f(1.0f, 1.0f, 1.0f));
  Handle(TDataStd_Name) name;
  
  if (shape_tool->IsShape(label)) {
    if (color_tool->IsSet(label, XCAFDoc_ColorSurf)) {
      color_tool->GetColor(label, color);
    }
    label.FindAttribute(TDataStd_Name::GetID(), name);
    auto name_str = (name.IsNull() ? TCollection_ExtendedString("UNDEFINED") : name->Get());
    std::cout << "\"" << name_str << "\": " 
              << "(" << color.Red() << ", " << color.Green() << ", " << color.Blue() << ")" 
              << "[" << Quantity_Color::StringName(color.Name()) << "]" << std::endl;
  } else {
    for (TDF_ChildIterator c(label); c.More(); c.Next()) {
      recursive(c.Value(), color_tool, shape_tool);
    }
  }
}

int main(int argc, const char **argv) {
  OSD::SetSignal(false);
  std::filesystem::path file_path, save_dir;

  std::map<std::string, bool> is_specified;
  get_cl_args(argc, argv, is_specified, file_path, save_dir);

  if (is_specified["--help"] 
      || is_specified["-h"]) {
    std::cout << help_message() << std::endl;
    return 0;
  }

  std::filesystem::create_directories(save_dir);
  auto name = file_path.filename();
  name.replace_extension("");

  std::optional<Statistics> stats;
  std::optional<std::filesystem::path> stl_path;
  std::optional<std::ofstream> nurbs_out;
  std::optional<TopoDS_CompSolid> conv_shape, conv_shape_no_trim;
  if (is_specified["--log_fails"]) {
    stats = Statistics{};
  }
  if (!is_specified["--no_nurbs"]) {
    auto nurbs_name = name;
    nurbs_name.replace_extension(".nurbs");
    auto nurbs_path = save_dir / nurbs_name;
    nurbs_out = std::ofstream(nurbs_path, std::ios::binary);
    nurbs_out.value().write("VERSION 300", 11);
  }
  if (is_specified["--brep"]) {
    conv_shape = TopoDS_CompSolid();
    BRep_Builder builder;
    builder.MakeCompSolid(conv_shape.value());
  }
  if (is_specified["--brep_no_trim"]) {
    conv_shape_no_trim = TopoDS_CompSolid();
    BRep_Builder builder;
    builder.MakeCompSolid(conv_shape_no_trim.value());
  }

  if (file_path.extension() == ".step"
      || file_path.extension() == ".stp") {
    Handle(TDocStd_Application) app = new TDocStd_Application;
    Handle(TDocStd_Document) doc;
    app->NewDocument("BinXCAF", doc);

    STEPCAFControl_Controller::Init();
    STEPControl_Controller::Init();

    STEPCAFControl_Reader reader;
    reader.SetColorMode(true);
    reader.SetNameMode(true);
    reader.SetLayerMode(true);
    reader.SetPropsMode(true);
    IFSelect_ReturnStatus status = reader.ReadFile(file_path.c_str());
    if (status != IFSelect_RetDone) {
      throw std::runtime_error("CAF Reader returned code "+std::to_string(status));
    }

    MyProgressIndicator indicator;
    Message_ProgressRange range = indicator.Start();
    std::thread progress_th(progress_bar, std::reference_wrapper(indicator), "Transferring roots from step...");
    if (!reader.Transfer(doc, range)) {
      throw std::runtime_error("CAF Reader could not translate file");
    }
    progress_th.join(); 

    auto root = doc->Main();
    auto color_tool = XCAFDoc_DocumentTool::ColorTool(root);
    auto shape_tool = XCAFDoc_DocumentTool::ShapeTool(root);
    recursive(root, color_tool, shape_tool);

  } else if (file_path.extension() == ".brep") {
    BRep_Builder builder;
    TopoDS_Shape shape;

    MyProgressIndicator indicator;
    Message_ProgressRange range = indicator.Start();
    std::thread progress_th(progress_bar, std::reference_wrapper(indicator), "Reading .brep file...");
    BRepTools::Read(shape, file_path.c_str(), builder, range);
    progress_th.join();

    int solid_id = 0, shapes_total = 0;
    for (TopExp_Explorer ex(shape, TopAbs_SOLID); ex.More(); ex.Next()) { 
      ++shapes_total;
    }
    for (TopExp_Explorer ex(shape, TopAbs_SOLID); ex.More(); ex.Next()) {
      if (!is_specified["--no_stl"]) {
        stl_path = save_dir / (std::to_string(solid_id) + ".stl");
      }
      process_solid(solid_id, shapes_total, TopoDS::Solid(ex.Current()), nurbs_out, stl_path, stats, conv_shape, conv_shape_no_trim);
      ++solid_id;
    }
  } else {
    throw std::invalid_argument(
        std::string("Incorrect format (")
      + file_path.extension().string()
      + "). Must be one of { .step, .stp, .brep }");
  }

  if (conv_shape) {
    std::cout << "Writing converted model to .brep..." << std::flush;
    auto conv_path = save_dir / (name.string()+"_conv.brep");
    BRepTools::Write(conv_shape.value(), conv_path.c_str());
    std::cout << "Done." << std::endl;
  }

  if (conv_shape_no_trim) {
    std::cout << "Writing untrimmed converted model to .brep..." << std::flush;
    auto conv_path = save_dir / (name.string()+"_conv_notrim.brep");
    BRepTools::Write(conv_shape_no_trim.value(), conv_path.c_str());
    std::cout << "Done." << std::endl;
  }

  if (is_specified["--log_fails"]) {
    std::filesystem::create_directories(save_dir / "Fails");
    auto &stats_ref = stats.value();
    for (auto &[name, solid]: stats_ref.failed_solids) {
      auto path = save_dir / "Fails" / name;
      BRepTools::Write(solid, path.c_str());
    }
    std::ofstream ffails(save_dir / "Fails" / "fails.txt");
    ffails << "Fails: " << stats_ref.fails.size() << std::endl;
    for (auto &fail: stats_ref.fails) {
      ffails << fail << std::endl;
    }
  }

  return 0;
}