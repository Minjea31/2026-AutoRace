#include "limo_base/steering_command_model.hpp"
#include <iostream>
#include <limits>

static void expect(double actual, double wanted) {
  if (std::abs(actual - wanted) > 1e-12)
    throw std::runtime_error("Unexpected steering command");
}

int main() {
  try {
    AgileX::SteeringCommandModel model;
    model.validate();
    for (double angle : {-1.2, -0.2, 0.0, 0.2, 1.2})
      expect(model.encode(angle, 2.47), angle / 2.47);
    model.wire_offset = 0.04;
    model.validate();
    expect(model.encode(0.0, 2.47), 0.04);
    expect(model.encode(0.247, 2.47), 0.14);
    expect(model.encode(-0.247, 2.47), -0.06);
    expect(model.encode(-0.0988, 2.47), 0.0);
    expect(model.encode(1.2, 2.47), 0.35);
    expect(model.encode(-1.2, 2.47), -0.35);
    model.enabled = true;
    expect(model.encode(0.0, 2.47), 0.04);
    expect(model.encode(0.1, 2.47), (0.1 + 0.23) / 1.92 + 0.04);
    expect(model.encode(-0.1, 2.47), -(0.1 + 0.23) / 1.92 + 0.04);
    model.wire_offset = 0.0;
    expect(model.encode(0.0, 2.47), 0.0);
    expect(model.encode(0.1, 2.47), (0.1 + 0.23) / 1.92);
    for (double invalid : {0.36, -0.36, std::numeric_limits<double>::quiet_NaN()}) {
      model.wire_offset = invalid;
      bool rejected = false;
      try { model.validate(); } catch (const std::invalid_argument&) { rejected = true; }
      if (!rejected) throw std::runtime_error("Invalid offset accepted");
    }
    std::cout << "Default/06 encoding, 04 neutral trim, both steering signs, limits, gain calibration and invalid offsets passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
