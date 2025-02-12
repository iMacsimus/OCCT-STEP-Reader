#include <iostream>
#include <cassert>
#include <fstream>
#include <format>
#include <array>
#include <chrono>
#include <thread>
#include <filesystem>

#include <vector>
#include <iostream>
#include <map>
namespace rv = std::ranges::views;

#include "common.hpp"

void process_solid(
    TopoDS_Solid solid) {
  //TODO
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

  MyProgressIndicator indicator;
  Message_ProgressRange range = indicator.Start();

  if (file_path.extension() == ".step"
      || file_path.extension() == ".stp") {
    STEPControl_Reader reader;
    IFSelect_ReturnStatus stat = reader.ReadFile(file_path.c_str());
    reader.PrintCheckLoad(true, IFSelect_PrintCount::IFSelect_ListByItem);
    
    std::thread progress_th(progress_bar, std::reference_wrapper(indicator), "Transferring roots from step...");
    reader.TransferRoots(range);
    progress_th.join(); 

    auto shapes_for_transfer = reader.NbShapes();
    for (int i: rv::iota(1, shapes_for_transfer)) {
      auto full_shape = reader.Shape(i);
      for (TopExp_Explorer ex(full_shape, TopAbs_SOLID); ex.More(); ex.Next()) {
        process_solid(TopoDS::Solid(ex.Current()));
      }
    }
  } else if (file_path.extension() == ".brep") {
    BRep_Builder builder;
    TopoDS_Shape shape;

    std::thread progress_th(progress_bar, std::reference_wrapper(indicator), "Reading .brep file...");
    BRepTools::Read(shape, file_path.c_str(), builder, range);
    progress_th.join();

    for (TopExp_Explorer ex(shape, TopAbs_SOLID); ex.More(); ex.Next()) {
      process_solid(TopoDS::Solid(ex.Current()));
    }
  } else {
    throw std::invalid_argument(
        std::string("Incorrect format (")
      + file_path.extension().string()
      + "). Must be one of { .step, .stp, .brep }");
  }

  return 0;
}