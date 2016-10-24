#ifndef GROUND_SEGMENTATION_SEGMENT_H_
#define GROUND_SEGMENTATION_SEGMENT_H_

#include <list>

#include "ground_segmentation/bin.h"

class Segment {
public:
  typedef std::pair<Bin::MinZPoint, Bin::MinZPoint> Line;

private:
  std::vector<Bin> bins_;

  std::list<Line> lines_;

public:

  Segment(const unsigned int n_bins);

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
