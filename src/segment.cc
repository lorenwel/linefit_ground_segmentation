#include "ground_segmentation/segment.h"

Segment::Segment(const unsigned int n_bins) : bins_(n_bins) {}

void Segment::fitSegmentLines() {
  // Find first point.
  auto line_start = bins_.begin();
  while (!line_start->hasPoint()) {
    ++line_start;
    // Stop if we reached last point.
    if (line_start == bins_.end()) return;
  }
  // Fill lines.
  for (auto line_end = line_start+1; line_end != bins_.end(); ++line_end) {
    if (line_end->hasPoint()) {
      lines_.emplace_back(line_start->getMinZPoint(), line_end->getMinZPoint());
      line_start = line_end;
    }
  }
}

bool Segment::getLines(std::list<Line> *lines) {
  if (lines_.empty()) {
    return false;
  }
  else {
    *lines = lines_;
    return true;
  }
}
