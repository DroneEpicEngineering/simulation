#include "trajectory_reader.hpp"
#include <stdexcept>

namespace target_system {
TrajectoryReader::TrajectoryReader() {}

void TrajectoryReader::read(const std::string &filename) {
  if (!reader_.mmap(filename)) {
    throw std::runtime_error("");
  }

  std::vector<TrajectoryPoint> trajectory;

  const auto header = reader_.header();
  for (const auto row : reader_) {
    TrajectoryPoint tp{};
    int cell_itr{};

    for (const auto cell : row) {
      std::string value;
      cell.read_value(value);

      switch (cell_itr) {
      case 0:
        tp.time = std::stod(value);
        break;
      case 1:
        tp.pos_x = std::stod(value);
        break;
      case 2:
        tp.pos_y = std::stod(value);
        break;
      case 3:
        tp.pos_z = std::stod(value);
        break;
      default:
        break;
      }

      trajectory.push_back(tp);
      cell_itr++;
    }
  }

  trajectory_ = trajectory;
  is_trajectory_read_ = true;
}

TrajectoryPoint TrajectoryReader::next_point() {
  return trajectory_.at(trajectory_index_++);
}

bool TrajectoryReader::is_trajectory_finished() const {
  return trajectory_index_ >= trajectory_.size(); 
}

bool TrajectoryReader::is_trajectory_read() const {
  return is_trajectory_read_;
}
} // namespace target_system