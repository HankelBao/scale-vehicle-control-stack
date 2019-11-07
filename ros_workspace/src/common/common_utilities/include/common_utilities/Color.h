#pragma once

#include <iostream>

namespace common_utilities {
class Color {
private:
  enum Type { RGB };

  uint8_t r_, g_, b_;

public:
  Color() : r_(0), g_(0), b_(0) {}
  Color(uint8_t r, uint8_t g, uint8_t b) : r_(r), g_(g), b_(b) {}

  uint8_t &r() { return r_; }
  uint8_t &g() { return g_; }
  uint8_t &b() { return b_; }

  void print() {
    std::cout << "(" << (int)r_ << ", " << (int)g_ << ", " << (int)b_ << ")"
              << std::endl;
  }
};

class ColorRange {
private:
  Color min_, max_;

public:
  ColorRange(Color min, Color max) : min_(min), max_(max) {}

  Color &min() { return min_; }
  Color &max() { return max_; }

  bool WithinRange(Color &other) {
    return other.r() > min_.r() && other.r() < max_.r() &&
           other.g() > min_.g() && other.g() < max_.g() &&
           other.b() > min_.b() && other.b() < max_.b();
  }
};
} // namespace common_utilities
