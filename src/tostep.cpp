#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <array>
#include <numeric>

// Function to write a Cartesian point
std::string writeCartesianPoint(int id, double x, double y, double z) {
    std::ostringstream oss;
    oss << "#" << id << " = CARTESIAN_POINT ( '', (" << x << ", " << y << ", " << z << ") );\n";
    return oss.str();
}

// Function to write a B-spline surface with knots
std::string writeBSplineSurfaceWithKnots(int id, const std::vector<int>& pointIds, 
                                         const std::vector<float>& weights, 
                                         int uDegree, int vDegree, 
                                         const std::vector<float>& uKnots, 
                                         const std::vector<int> &uMults,
                                         const std::vector<float>& vKnots, 
                                         const std::vector<int> &vMults) {
    std::ostringstream oss;
    oss << "#" << id << " = B_SPLINE_SURFACE_WITH_KNOTS ( '', " 
        << uDegree << ", " << vDegree << ", (";

    for (size_t i = 0; i < pointIds.size(); ++i) {
        oss << "#" << pointIds[i];
        if (i < pointIds.size() - 1) oss << ", ";
    }

    oss << "), .UNSPECIFIED., .F., .F., (";

    for (size_t i = 0; i < weights.size(); ++i) {
        oss << weights[i];
        if (i < weights.size() - 1) oss << ", ";
    }

    oss << "), (";

    for (size_t i = 0; i < uKnots.size(); ++i) {
        oss << uKnots[i];
        if (i < uKnots.size() - 1) oss << ", ";
    }

    oss << "), (";

    for (size_t i = 0; i < vKnots.size(); ++i) {
        oss << vKnots[i];
        if (i < vKnots.size() - 1) oss << ", ";
    }

    oss << "), (";

    for (size_t i = 0; i < uKnots.size(); ++i) {
        oss << uMults[i];
        if (i < uKnots.size() - 1) oss << ", ";
    }

    oss << "), (";

    for (size_t i = 0; i < vKnots.size(); ++i) {
        oss << vMults[i];
        if (i < vKnots.size() - 1) oss << ", ";
    }

    oss << ") );\n";

    return oss.str();
}

using float3 = std::array<float, 3>;

void nurbss2step(std::ifstream &fin, std::ofstream &stepFile) {
    // Write header
    stepFile << "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('NURBS Surface'),'2;1');\nENDSEC;\nDATA;\n";

    char tmp_c;
    std::string tmp_s;
    int n, m, u_deg, v_deg;
    int id_offset = 1;
    while(fin >> tmp_c >> tmp_c >> n >> tmp_c >> tmp_c >> m) {
      std::vector<int> points_ids((n+1)*(m+1));
      std::iota(points_ids.begin(), points_ids.end(), id_offset);
      std::vector<float3> points((n+1)*(m+1));
      std::vector<float> weights((n+1)*(m+1));
      std::vector<float> u_knots, v_knots;
      std::vector<int> u_mults, v_mults;
      
      fin >> tmp_s;
      for (auto &point: points) {
        fin >> tmp_c >> point[0] >> tmp_c >> point[1] >> tmp_c >> point[2] >> tmp_c;
        stepFile << writeCartesianPoint(id_offset++, point[0], point[1], point[2]);
      }

      fin >> tmp_s;
      for (auto &weight: weights) {
        fin >> weight;
      }

      fin >> tmp_s >> u_deg;
      fin >> tmp_s >> v_deg;

      fin >> tmp_s;
      for (int i = 0; i < n+u_deg+2; ++i) {
        float uknot;
        fin >> uknot;
        if (u_knots.empty() || u_knots.back() < uknot) {
          u_knots.push_back(uknot);
          u_mults.push_back(1);
        } else {
          ++u_mults.back();
        }
      } 

      fin >> tmp_s;
      for (int i = 0; i < m+v_deg+2; ++i) {
        float vknot;
        fin >> vknot;
        if (v_knots.empty() || v_knots.back() < vknot) {
          v_knots.push_back(vknot);
          v_mults.push_back(1);
        } else {
          ++v_mults.back();
        }
      } 

      stepFile << writeBSplineSurfaceWithKnots(
        id_offset++, points_ids, 
        weights, u_deg, v_deg, 
        u_knots, u_mults,
        v_knots, v_mults);
    }

    // Write footer
    stepFile << "ENDSEC;\nEND-ISO-10303-21;\n";

    std::cout << "STEP file written successfully.\n";
}

int main(int argc, char **argv) {
  std::ifstream fin(argv[1]);
  std::ofstream fout(std::string(argv[2])+".step");

  nurbss2step(fin, fout);
}
