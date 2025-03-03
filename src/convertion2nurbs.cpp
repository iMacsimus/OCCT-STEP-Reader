#include <cassert>
#include <thread>

#include "common.hpp"

template <typename T> void binout(std::ostream &fout, const T &value) {
  fout.write(reinterpret_cast<const char *>(&value), sizeof(T));
}

template <typename T> void binout(std::ostream &fout, const T *pvalues, size_t count) {
  fout.write(reinterpret_cast<const char *>(pvalues), count * sizeof(T));
}

void output_curve(Geom2d_BSplineCurve *bspline, Standard_Real umin,
                  Standard_Real umax, std::ostream &fout) {
  if (bspline->IsPeriodic()) {
    bspline->SetNotPeriodic();
  }

  Standard_Real firstKnot = *bspline->Knots().begin();
  Standard_Real lastKnot = *(bspline->Knots().end()-1);
  assert(firstKnot < umin+1e-5);
  assert(lastKnot > umax-1e-5);

  int n = bspline->NbPoles() - 1;
  binout(fout, n);

  for (auto &point : bspline->Poles()) {
    float point_values[3] = {static_cast<float>(point.X()),
                             static_cast<float>(point.Y()), 1.0f};
    binout(fout, point_values);
  }

  if (bspline->Weights() != nullptr) {
    for (float weight : *bspline->Weights()) {
      binout(fout, weight);
    }
  } else {
    for (int i = 0; i < (n + 1); ++i) {
      float weight = 1.0f;
      binout(fout, weight);
    }
  }

  int u_deg = bspline->Degree();
  binout(fout, u_deg);

  binout(fout, bspline->Knots().Size());
  for (float knot : bspline->Knots())
    binout(fout, knot);
  for (int mult : bspline->Multiplicities())
    binout(fout, mult);

  binout(fout, static_cast<float>(umin));
  binout(fout, static_cast<float>(umax));

  assert(std::invoke([&]() {
    auto mults = bspline->Multiplicities();
    int knots_count = std::accumulate(mults.begin(), mults.end(), 0);
    return knots_count == n + u_deg + 2;
  }));
}

void output_nurbs(Geom_BSplineSurface *bspline, std::ostream &fout) {
  if (bspline->IsUPeriodic()) {
    bspline->SetUNotPeriodic();
  }
  if (bspline->IsVPeriodic()) {
    bspline->SetVNotPeriodic();
  }

  int n = bspline->NbUPoles() - 1, m = bspline->NbVPoles() - 1;
  binout(fout, n);
  binout(fout, m);

  for (auto &point : bspline->Poles()) {
    float point_values[4] = {static_cast<float>(point.X()),
                             static_cast<float>(point.Y()),
                             static_cast<float>(point.Z()), 1.0f};
    binout(fout, point_values);
  }

  if (bspline->Weights() != nullptr) {
    for (float weight : *bspline->Weights()) {
      binout(fout, weight);
    }
  } else {
    for (int i = 0; i < (n + 1) * (m + 1); ++i) {
      float weight = 1.0f;
      binout(fout, weight);
    }
  }

  int u_deg = bspline->UDegree(), v_deg = bspline->VDegree();
  binout(fout, u_deg);
  binout(fout, v_deg);

  binout(fout, bspline->UKnots().Size());
  for (float knot : bspline->UKnots()) {
    binout(fout, knot);
  }
  for (int mult : bspline->UMultiplicities()) {
    binout(fout, mult);
  }

  binout(fout, bspline->VKnots().Size());
  for (float knot : bspline->VKnots()) {
    binout(fout, knot);
  }
  for (int mult : bspline->VMultiplicities()) {
    binout(fout, mult);
  }
}

auto convert_shape(TopoDS_Shape &shape,
                   const TCollection_ExtendedString &name) {
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
    GeomAbs_SurfaceType type;
    {
      BRepAdaptor_Surface surface(TopoDS::Face(ex.Current()));
      type = surface.GetType();
    }
    std::cout << "Converting \"" << name << "\"'s " << total
              << "th face (" << geom_abs2str[type] << ") to Bspline..." << std::flush;
    auto face = ex.Current();
    try {
      OCC_CATCH_SIGNALS
      //if (type != GeomAbs_BSplineSurface) {
        convertor.Perform(face, true);
        face = convertor.Shape();
      //}
      BRepAdaptor_Surface surface(TopoDS::Face(face));
      auto type = surface.GetType();
      if (type != GeomAbs_BSplineSurface) {
        std::string message = std::string("Type after convertion is not bspline surface (got ")+geom_abs2str[type]+")";
        throw Standard_Failure(message.c_str());
      }
      faces.push_back(TopoDS::Face(face));
    } catch (Standard_Failure &err) {
      std::cout << "Failed. Skip." << std::endl;
      std::cerr << name + ": " << err << std::endl;
      continue;
    } catch (...) {
      std::cout << "Failed. Skip." << std::endl;
      std::cerr << name + ": " << "Unknown" << std::endl;
      continue;
    }
    ++total;
    std::cout << "Done." << std::endl;
  }

  return faces;
}

void convert2nurbs(const TCollection_ExtendedString &name, TopoDS_Shape shape,
                   Quantity_Color color,
                   std::ofstream &fout) {
  BRep_Builder builder;

  auto faces = convert_shape(shape, name);
  binout(fout, static_cast<int>(faces.size()));

  std::cout << "Output \"" << name << "\" to .nurbs..." << std::flush;
  TCollection_AsciiString name_ascii = name;
  std::string name_str = name_ascii.ToCString();
  binout(fout, static_cast<int>(name_str.size()));
  binout(fout, name_str.c_str(), name_str.size());
  binout(fout, static_cast<float>(color.Red()));
  binout(fout, static_cast<float>(color.Green()));
  binout(fout, static_cast<float>(color.Blue()));
  for (auto &face : faces) {
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
      TopoDS_Shape edge = exp.Current();
      Standard_Real parL, parF;
      auto curve = BRep_Tool::Curve(TopoDS::Edge(edge), parL, parF);
      BRepAdaptor_Curve2d adaptor(TopoDS::Edge(edge), face);
      //assert(adaptor.GetType() == GeomAbs_BSplineCurve);
      auto handler = adaptor.BSpline();
      auto bspline = handler.get();
      output_curve(bspline, parL, parF, fout);
    }
  }
  std::cout << "Done." << std::endl;
}