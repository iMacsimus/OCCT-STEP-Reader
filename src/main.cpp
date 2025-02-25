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
#include <unordered_map>
#include <optional>
#include <set>
#include <tuple>

#include "common.hpp"

constexpr const char *shape2str[] = {
  "TopAbs_COMPOUND",
  "TopAbs_COMPSOLID",
  "TopAbs_SOLID",
  "TopAbs_SHELL",
  "TopAbs_FACE",
  "TopAbs_WIRE",
  "TopAbs_EDGE",
  "TopAbs_VERTEX",
  "TopAbs_SHAPE"
};

struct ColorInfo
{
  Quantity_Color rgb = Quantity_NOC_WHITE;
  XCAFDoc_ColorType type = XCAFDoc_ColorGen;
  bool operator<(ColorInfo other) const {
    return std::make_tuple(rgb.Red(), rgb.Green(), rgb.Blue(), type) <
           std::make_tuple(other.rgb.Red(), other.rgb.Green(), other.rgb.Blue(), other.type);
  }
};

class Converter
{
public:
  bool do_obj = true;
  bool do_nurbs = true;
  std::filesystem::path save_dir;
public:
  void process_shape(
    const TopoDS_Shape &shape,
    Quantity_Color color,
    const TCollection_ExtendedString&name);
  void process(Handle(TDocStd_Document) doc);
private:
  int undef_counter = 0;
  int cur_id = 0;
  void process_recursive(
    const TDF_Label &label, 
    Handle(XCAFDoc_ColorTool) color_tool, 
    Handle(XCAFDoc_ShapeTool) shape_tool,
    std::unordered_map<TDF_Label, ColorInfo> &cached,
    ColorInfo parent_color = ColorInfo{},
    bool is_root = true);
};

void Converter::process_shape(
    const TopoDS_Shape &shape,
    Quantity_Color color,
    const TCollection_ExtendedString &name) {
  int counter = 0;
  auto cur_id_str = std::to_string(cur_id);
  for (TopExp_Explorer ex(shape, TopAbs_SOLID); ex.More(); ex.Next()) {
    if (do_obj) {
      auto counter_str = std::to_string(counter++);
      auto cur_name = name+"_#"+cur_id_str.c_str()+"_SOLID_"+counter_str.c_str()+".obj";
      std::ofstream table(save_dir / "name-col-type.txt", std::ios::app);
      table << cur_name << ", (" << color.Red() << ", " << color.Green() << ", " << color.Blue() << "), " 
            << shape2str[shape.ShapeType()] << ";" << std::endl;
      auto cur_dir = save_dir / cur_name.ToExtString();
      tesselate_shape(ex.Current(), cur_dir);
    }
    if (do_nurbs) {
      std::ofstream nurbs_out(save_dir/"converted.nurbs", std::ios::binary | std::ios::app);
      convert2nurbs(name, shape, nurbs_out);
    }
  }
  counter = 0;
  for (TopExp_Explorer ex(shape, TopAbs_SHELL, TopAbs_SOLID); ex.More(); ex.Next()) {
    if (do_obj) {
      auto counter_str = std::to_string(counter++);
      auto cur_name = name+"_#"+cur_id_str.c_str()+"_SHELL_"+counter_str.c_str()+".obj";
      std::ofstream table(save_dir / "name-col-type.txt", std::ios::app);
      table << cur_name << ", (" << color.Red() << ", " << color.Green() << ", " << color.Blue() << "), " 
            << shape2str[shape.ShapeType()] << ";" << std::endl;
      auto cur_dir = save_dir / cur_name.ToExtString();
      tesselate_shape(ex.Current(), cur_dir);
    }
    if (do_nurbs) {
      std::ofstream nurbs_out(save_dir/"converted.nurbs", std::ios::binary | std::ios::app);
      convert2nurbs(name, shape, nurbs_out);
    }
  }
  counter = 0;
  for (TopExp_Explorer ex(shape, TopAbs_FACE, TopAbs_SHELL); ex.More(); ex.Next()) {
    if (do_obj) {
      auto counter_str = std::to_string(counter++);
      auto cur_name = name+"_#"+cur_id_str.c_str()+"_FACE_"+counter_str.c_str()+".obj";
      std::ofstream table(save_dir / "name-col-type.txt", std::ios::app);
      table << cur_name << ", (" << color.Red() << ", " << color.Green() << ", " << color.Blue() << "), " 
            << shape2str[shape.ShapeType()] << ";" << std::endl;
      auto cur_dir = save_dir / cur_name.ToExtString();
      tesselate_shape(ex.Current(), cur_dir);
    }
    if (do_nurbs) {
      std::ofstream nurbs_out(save_dir/"converted.nurbs", std::ios::binary | std::ios::app);
      convert2nurbs(name, shape, nurbs_out);
    }
  }
  ++cur_id;
}

void Converter::process_recursive(
    const TDF_Label &label, 
    Handle(XCAFDoc_ColorTool) color_tool, 
    Handle(XCAFDoc_ShapeTool) shape_tool,
    std::unordered_map<TDF_Label, ColorInfo> &cached,
    ColorInfo parent_color,
    bool is_root) {
  ColorInfo my_color{};

  if (!color_tool->IsSet(label, XCAFDoc_ColorSurf)) {
    if (parent_color.type == XCAFDoc_ColorSurf) {
      my_color = parent_color;
    }
  } else {
    color_tool->GetColor(label, XCAFDoc_ColorSurf, my_color.rgb);
    my_color.type = XCAFDoc_ColorSurf;
  }

  if (cached.find(label) == cached.end()) {
    std::set<ColorInfo> child_colors;
    for (TDF_ChildIterator c(label); c.More(); c.Next()) {
      process_recursive(c.Value(), color_tool, shape_tool, cached, my_color, false);
      child_colors.insert(cached[c.Value()]);
    }

    if (child_colors.size() > 1) {
      cached[label] = ColorInfo{};
    } else if (child_colors.empty()) {
      cached[label] = my_color;
    } else {
      cached[label] = *child_colors.begin();
    }
  }

  if (!is_root) {
    return;
  }

  my_color = cached[label];
  
  if (shape_tool->IsShape(label)) {
    auto shape = shape_tool->GetShape(label);
    auto shape_type = shape.ShapeType();
    bool divide = (cached[label].type == XCAFDoc_ColorGen) && (label.NbChildren() != 0);

    if (!divide) {
      Handle(TDataStd_Name) maybe_name;
      label.FindAttribute(TDataStd_Name::GetID(), maybe_name);
      std::string untitled = "Untitled" + std::to_string(undef_counter);
      auto name = (maybe_name.IsNull() 
                                              ? TCollection_ExtendedString(untitled.c_str()) 
                                              : maybe_name->Get());
      if (name == untitled.c_str()) {
        ++undef_counter;
      }

      process_shape(shape, my_color.rgb, name);
      return;
    }
  }

  for (TDF_ChildIterator c(label); c.More(); c.Next()) {
    process_recursive(c.Value(), color_tool, shape_tool, cached, my_color, true);
  }
}

void Converter::process(Handle(TDocStd_Document) doc) {
  cur_id = 0;
  undef_counter = 0;
  auto root = doc->Main();
  auto color_tool = XCAFDoc_DocumentTool::ColorTool(root);
  auto shape_tool = XCAFDoc_DocumentTool::ShapeTool(root);
  std::unordered_map<TDF_Label, ColorInfo> cached;
  process_recursive(root, color_tool, shape_tool, cached);
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

  std::filesystem::remove_all(save_dir);
  std::filesystem::create_directories(save_dir);

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

    Converter converter;
    converter.do_nurbs = false; //!is_specified["--no_nurbs"];
    converter.do_obj = !is_specified["--no_obj"];
    converter.save_dir = save_dir;
    converter.process(doc);
  } else {
    throw std::invalid_argument(
        std::string("Incorrect format (")
      + file_path.extension().string()
      + "). Must be one of { .step, .stp }");
  }

  return 0;
}