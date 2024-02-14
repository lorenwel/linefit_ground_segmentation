#ifndef GROUND_SEGMENTATION_SEGMENT_H_
#define GROUND_SEGMENTATION_SEGMENT_H_

#include <list>
#include <map>

#include "ground_segmentation/bin.h"

class Segment {
public:
  typedef std::pair<Bin::MinZPoint, Bin::MinZPoint> Line;

  typedef std::pair<double, double> LocalLine;

private:
  // Parameters. Description in GroundSegmentation.
  const double min_slope_;
  const double max_slope_;
  const double max_error_;
  const double long_threshold_;
  const double max_long_height_;
  const double max_start_height_;
  const double sensor_height_;

  std::vector<Bin> bins_;

  std::list<Line> lines_;

  LocalLine fitLocalLine(const std::list<Bin::MinZPoint>& points);

  double getMeanError(const std::list<Bin::MinZPoint>& points, const LocalLine& line);

  double getMaxError(const std::list<Bin::MinZPoint>& points, const LocalLine& line);

  Line localLineToLine(const LocalLine& local_line, const std::list<Bin::MinZPoint>& line_points);


public:

  Segment(const unsigned int& n_bins,
          const double& min_slope,
          const double& max_slope,
          const double& max_error,
          const double& long_threshold,
          const double& max_long_height,
          const double& max_start_height,
          const double& sensor_height);

  double verticalDistanceToLine(const double& d, const double &z);

  void fitSegmentLines();

  inline Bin& operator[](const size_t& index) {
    return bins_[index];
  }

  inline std::vector<Bin>::iterator begin() {
    return bins_.begin();
  }

  inline std::vector<Bin>::iterator end() {
    return bins_.end();
  }

  bool getLines(std::list<Line>* lines);

};

#endif /* GROUND_SEGMENTATION_SEGMENT_H_ */
