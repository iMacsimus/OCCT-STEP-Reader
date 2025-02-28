#include <thread>
#include <cassert>

#include "common.hpp"

template<typename T>
void binout(std::ostream &fout, const T& value) {
  fout.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

std::vector<Standard_Real>
heal_knots(
    const TColStd_Array1OfReal &knots,
    const TColStd_Array1OfInteger &mults,
    size_t n, size_t p,
    Standard_Real umin, Standard_Real umax) {
  std::vector<std::pair<Standard_Real, size_t>> modified;
  {
    std::vector<Standard_Real> vec_knots;
    std::copy(knots.begin(), knots.end(), std::back_inserter(vec_knots));
    std::vector<size_t> vec_mults;
    std::copy(mults.begin(), mults.end(), std::back_inserter(vec_mults));
    for (size_t i = 0; i < vec_knots.size(); ++i) {
      modified.push_back({ vec_knots[i], vec_mults[i] });
    }
  }
  // std::sort(modified.begin(), modified.end());
  // for (int i = modified.size()-1; i >= 0; --i) {
  //   auto val = modified[i].first;
  //   if ((val < umin) || (val > umax)) {
  //     modified.erase(modified.begin()+i);
  //     --i;
  //   }
  // }
  // if (std::abs(modified[0].first-umin) > 1e-5) {
  //   modified.insert(modified.begin(), { umin, p+1 });
  // } else {
  //   modified[0].second = p+1;
  // }
  // if (std::abs(modified.back().first-umax) > 1e-5) {
  //   modified.push_back({ umax, p + 1 });
  // } else {
  //   modified.back().second = p+1;
  // }

  // int count = 2*(p+1);
  // for (int i = 1; i < modified.size()-1; ++i) {
  //   auto &[val, mult] = modified[i];
  //   if (mult > p) {
  //     mult = p;
  //   }
  //   count += mult;
  // }

  // for (auto &[val, mult]: modified) {
  //   if (count == n+p+2) {
  //     break;
  //   } else if (count < n+p+2 && mult < p+1) {
  //     size_t delta = std::min(n+p+2-count, p-mult);
  //     mult += delta;
  //     count += delta;
  //   } else if (count > n+p+2) {
  //     size_t delta = std::min(count-n-p-2, mult-1);
  //     mult -= delta;
  //     count -= delta;
  //   }
  // }

  std::vector<Standard_Real> result;
  for (auto &[val, mult]: modified) {
    result.insert(result.end(), mult, val);
  }

  for (size_t i = 0; i <= p; ++i) {
    result[i] = umin;
    result[result.size()-i-1] = umax;
  }

  assert(result.size() == n+p+2);
  assert(std::is_sorted(result.begin(), result.end()));
  assert(std::abs(result.front() - umin) < 1e-3);
  assert(std::abs(result.front() - umin) < 1e-3);
  assert(std::abs(result.back() - umax) < 1e-3);
  assert(std::abs(result[n+1] - umax) < 1e-3);
  return result;
}

void output_curve(Geom2d_BSplineCurve *bspline, std::ostream &fout) {
  if (bspline->IsPeriodic()) {
    bspline->SetNotPeriodic();
  }

  int n = bspline->NbPoles()-1;
  binout(fout, n);

  for (auto &point: bspline->Poles()) {
    float point_values[3] = { 
      static_cast<float>(point.X()), 
      static_cast<float>(point.Y()), 
      1.0f 
    };
    binout(fout, point_values);
  }
  
  if (bspline->Weights() != nullptr) {
    for (float weight: *bspline->Weights()) {
      binout(fout, weight);
    }
  } else {
    for (int i = 0; i < (n+1); ++i) {
      float weight = 1.0f;
      binout(fout, weight);
    }
  }

  int u_deg = bspline->Degree();
  binout(fout, u_deg);

  Standard_Real umin = bspline->FirstParameter(), umax = bspline->LastParameter();
  auto knots = heal_knots(bspline->Knots(), bspline->Multiplicities(), n, u_deg, umin, umax);
  for (float knot: knots) {
    binout(fout, knot);
  }
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

  Standard_Real umin, umax, vmin, vmax;
  bspline->Bounds(umin, umax, vmin, vmax);

  auto knots = heal_knots(bspline->UKnots(), bspline->UMultiplicities(), n, u_deg, umin, umax);
  for (float knot: knots) {
    binout(fout, knot);
  }

  knots = heal_knots(bspline->VKnots(), bspline->VMultiplicities(), m, v_deg, vmin, vmax);
  for (float knot: knots) {
    binout(fout, knot);
  }
}

auto convert_shape(TopoDS_Shape &shape, const TCollection_ExtendedString &name) {
  std::vector<TopoDS_Face> faces;
  BRepBuilderAPI_NurbsConvert convertor;
  
  ShapeUpgrade_ShapeDivideClosed div_shape{shape};
  div_shape.Perform();
  shape = div_shape.Result();
  ShapeUpgrade_ShapeDivideClosedEdges div_edges{shape};
  div_edges.Perform();
  shape = div_edges.Result();
  
  int total = 0;
  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    std::cout << "Converting \"" << name << "\"'s " << total << "th face to Bspline..." << std::flush;
    auto face = ex.Current();
    try {
      OCC_CATCH_SIGNALS
      convertor.Perform(face, true);
      face = convertor.Shape();
      BRepAdaptor_Surface surface(TopoDS::Face(face));
      auto type = surface.GetType();
      if (type != GeomAbs_BSplineSurface) {
        throw Standard_Failure("Type after convertion is not bspline surface");
      }
      faces.push_back(TopoDS::Face(face));
    } catch(Standard_Failure &err) {
      std::cout << "Failed. Skip." << std::endl;
      std::cerr << name + ": " << err << std::endl;
      continue;
    } catch(...) {
      std::cout << "Failed. Skip." << std::endl;
      std::cerr << name + ": " << "Unknown" << std::endl;
      continue;
    }
    ++total;
    std::cout <<"Done." << std::endl;
  }

  return faces;
}

void convert2nurbs(
      const TCollection_ExtendedString &name,
      TopoDS_Shape shape,
      std::ofstream &fout) {
  BRep_Builder builder;

  auto faces = convert_shape(shape, name);
  binout(fout, static_cast<int>(faces.size()));

  std::cout << "Output \"" << name << "\" to .nurbs..." << std::flush;
  for (auto &face: faces) {
    BRepAdaptor_Surface surface(face);
    auto type = surface.GetType();
    assert(type == GeomAbs_BSplineSurface);
    {
      auto bspline_handler = surface.BSpline();
      auto bspline = bspline_handler.get();
      output_nurbs(bspline, fout);
    }
    
    int curves_count = 0;
    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
      ++curves_count;
    }
    binout(fout, curves_count);

    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
      TopoDS_Edge edge = TopoDS::Edge(exp.Current());
      BRepAdaptor_Curve2d adaptor(edge, face);
      auto handler = adaptor.BSpline();
      auto bspline = handler.get();
      output_curve(bspline, fout);
    }
  }
  std::cout << "Done." << std::endl;
}