#include "common.hpp"

std::map<std::string, bool> is_required = {
  { "--help", false }, 
  { "-h", false },
  { "--file_path", true },
  { "--save_dir", true },
  { "--no_nurbs", false },
  { "--no_obj", false },
  { "--deflection", false}
};

const char help_message_cstr[] = 
R"(OCCT STEP Reader
------------------------------------------------------------
Reads and parses STEP or .brep files, tesselates unicolored 
parts and export as .obj file, specific for
each solid. Also, converts all geometry to bspline curves
ans surfaces and exports as .nurbs (poprietary LiteRT format) 
file, one for the whole model.
------------------------------------------------------------
!!! Required Arguments:
* --file_path <path> - path to original file
(Supported extensions: .step, .stp, .brep);
* --save_dir <path> - directory to save output files;
------------------------------------------------------------
Optional:
* --no_nurbs: disable .nurbs file generation
* --no_obj: disable .obj files generation
* --help or -h: description of the command-line options
understood by OCCT STEP Reader.
* --deflection <value> - set deflection for 
tesselated mesh

Arguments can be combined without any restriction. If 
--help or -h specified, only help message will be printed
without output of any files.
------------------------------------------------------------
Output files:
* <name>.obj, name-col-type.txt, converted.nurbs - .obj for 
some part, table with color and type of part, 
.nurbs for the whole model
)";

void get_cl_args(
    int argc, const char **argv,
    std::map<std::string, bool> &is_specified,
    std::filesystem::path &file_path,
    std::filesystem::path &save_dir,
    Standard_Real &deflection) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (is_required.find(arg) == is_required.end()) {
      throw std::invalid_argument(std::string("invalid argument: ")+arg);
    }
    is_specified[arg] = true;
    if (arg == "--file_path") {
      file_path = argv[i+1];
      ++i;
    }
    if (arg == "--save_dir") {
      save_dir = argv[i+1];
      ++i;
    }
    if (arg == "--deflection") {
      std::stringstream ss;
      ss << argv[i+1];
      ss >> deflection;
    }
  }
  if (!is_specified["--help"] && !is_specified["-h"]) {
    for (auto &[arg, is_req]: is_required) {
      if (is_req && !is_specified[arg]) {
        throw std::invalid_argument(arg+" is not specified");
      }
    }
  }
}

std::string help_message() {
  return help_message_cstr;
}