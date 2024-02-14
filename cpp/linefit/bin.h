#pragma once
#include <atomic>

class Bin {
public:
  struct MinZPoint {
    MinZPoint() : z(0), d(0) {}
    MinZPoint(const double &d, const double &z) : z(z), d(d) {}
    bool operator==(const MinZPoint &comp) {
      return z == comp.z && d == comp.d;
    }

    double z;
    double d;
  };

private:
  std::atomic<bool> has_point_;
  std::atomic<double> min_z;
  std::atomic<double> min_z_range;

public:
  Bin() : min_z(std::numeric_limits<double>::max()), has_point_(false){};

  /// \brief Fake copy constructor to allow vector<vector<Bin> > initialization.
  Bin(const Bin &segment)
      : min_z(std::numeric_limits<double>::max()), has_point_(false){};

  void addPoint(const double &d, const double &z) {
    has_point_ = true;
    if (z < min_z) {
      min_z = z;
      min_z_range = d;
    }
  };

  MinZPoint getMinZPoint() {
    MinZPoint point;

    if (has_point_) {
      point.z = min_z;
      point.d = min_z_range;
    }

    return point;
  };

  inline bool hasPoint() { return has_point_; }
};
