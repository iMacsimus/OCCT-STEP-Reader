#include "common.hpp"

std::map<std::string, bool> is_required = {
  { "--help", false }, 
  { "-h", false },
  { "--file_path", true },
  { "--save_dir", true },
  { "--brep_no_trim", false },
  { "--brep", false },
  { "--log_fails", false },
  { "--no_nurbs", false },
  { "--no_stl", false }
};

const char help_message_cstr[] = 
R"(OCCT STEP Reader
------------------------------------------------------------
Reads and parses STEP or .brep files, tesselates solid 
watertight parts and export as .stl file, specific for
each solid. Also, converts all faces to bspline surfaces
and exports as .nurbs (poprietary LiteRT format) file,
one for the whole model.
------------------------------------------------------------
!!! Required Arguments:
* --file_path <path> - path to original file
(Supported extensions: .step, .stp, .brep);
* --save_dir <path> - directory to save output files;
------------------------------------------------------------
Optional:
* --no_nurbs: disable .nurbs file generation
* --no_stl: disable .stl files generation
* --brep_no_trim: additionally save .brep file after
convertation of all faces to bspline surface,
except trimming curves;
* --brep: additionally save .brep file after convertation 
of all faces to bspline surfaces, including trimming 
curves;
* --log_fails: save dumps of failed to convert faces 
  and their error messages;
* --help or -h: description of the command-line options
understood by OCCT STEP Reader.

Arguments can be combined without any restriction. If 
--help or -h specified, only help message will be printed
without output of any files.
------------------------------------------------------------
Output files:
* <number>.stl, <filename>.nurbs - .stl for <number>-th 
solid part & .nurbs for the whole model
* <filename>_conv_notrim.brep (Optional) - output for 
"--brep_no_trim" argument;
* <filename>_conv.brep (Optional) - output for "--brep" argument;
* Fails/<number>.brep (Optional) - dump of <number>-th 
failed face;
* Fails/fails.txt (Optional) - error messages for each
failed face;
* Fails/stats.txt (Optional) - number of faces for each type.
)";

void get_cl_args(
    int argc, const char **argv,
    std::map<std::string, bool> &is_specified,
    std::filesystem::path &file_path,
    std::filesystem::path &save_dir) {
  is_specified = {
    { "--help", false }, 
    { "-h", false },
    { "--file_path", false },
    { "--save_dir", false },
    { "--brep_no_trim", false },
    { "--brep", false },
    { "--log_fails", false },
    { "--no_nurbs", false },
    { "--no_stl", false }
  };

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (is_specified.find(arg) == is_specified.end()) {
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