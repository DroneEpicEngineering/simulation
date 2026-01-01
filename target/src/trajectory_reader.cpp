#include "trajectory_reader.hpp"

#include <iostream>
#include <stdexcept>

namespace target_system {
TrajectoryReader::TrajectoryReader() {}

void TrajectoryReader::read(const std::string &filename) {
  if (!reader_.mmap(filename)) {
    throw std::runtime_error("");
  }

  std::vector<TrajectoryPoint> trajectory;

  const auto header = reader_.header();
  bool first_row = true;

  for (const auto row : reader_) {
    if (first_row) 
    {
      first_row = false;
      continue;
    }

    TrajectoryPoint tp{};
    int cell_itr{};

    for (const auto cell : row) {
      std::string value;
      cell.read_value(value);

      switch (cell_itr) {
      case 0:
        tp.time = value.empty() ? 0.0 : std::stod(value);
        break;
      case 1:
        tp.pos_x = value.empty() ? 0.0 : std::stod(value);
        break;
      case 2:
        tp.pos_y = value.empty() ? 0.0 : std::stod(value);
        break;
      case 3:
        tp.pos_z = value.empty() ? 0.0 : std::stod(value);
        break;
      case 4:
        tp.vel_x = value.empty() ? 0.0 : std::stod(value);
        break;
      case 5:
        tp.vel_y = value.empty() ? 0.0 : std::stod(value);
        break;
      case 6:
        tp.vel_z = value.empty() ? 0.0 : std::stod(value);
        break;
      default:
        break;
      }

      cell_itr++;
    }
    trajectory.push_back(tp);
  }

  trajectory_ = trajectory;
  is_trajectory_read_ = true;
}

TrajectoryPoint TrajectoryReader::next_point() {
  return trajectory_.at(trajectory_index_++);
}

bool TrajectoryReader::is_trajectory_finished() const {
  return is_trajectory_read_ && trajectory_index_ >= trajectory_.size();
}

bool TrajectoryReader::is_trajectory_read() const {
  return is_trajectory_read_;
}
} // namespace target_system