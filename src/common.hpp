#include <map>
#include <filesystem>
#include <iomanip>
#include <string>
#include <thread>
#include <ranges>
#include <type_traits>
#include <optional>

#include "occt_headers.hpp"

void get_cl_args(
    int argc, const char **argv,
    std::map<std::string, bool> &is_specified,
    std::filesystem::path &file_path,
    std::filesystem::path &save_dir);
std::string help_message();

class MyProgressIndicator : public Message_ProgressIndicator {
public:
    MyProgressIndicator() {}
protected:
    virtual void Show(const Message_ProgressScope& theScope, 
                     const Standard_Boolean isForce) override {}
};

inline
void progress_bar(Message_ProgressIndicator &indicator, std::string message) {
  while((1.0 - indicator.GetPosition()) > 1e-4) {
    std::cout << '\r' << message << " " << std::setw(10) << indicator.GetPosition()*100 << "%   " << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << std::setw(0);
  if (true)
    std::cout << "\r" << message << " Done.                 " << std::endl;
  else 
    std::cout << "\r" << message << "Failed.                " << std::endl;
}

constexpr std::array geom_abs2str = {
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

struct Statistics
{
  std::vector<std::string> fails;
  std::vector<std::pair<std::string, TopoDS_Solid>> failed_solids;
};

void tesselate_solid(const TopoDS_Solid& shape, std::filesystem::path save_path);
void convert2nurbs(
      int shape_id, int shapes_total,
      TopoDS_Solid shape, 
      std::optional<std::ofstream> &fout,
      std::optional<Statistics> &stats,
      std::optional<TopoDS_CompSolid> &conv_shape,
      std::optional<TopoDS_CompSolid> &conv_shape_notrim);