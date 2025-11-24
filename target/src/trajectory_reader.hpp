#include <string>
#include <vector>

#include <csv2/reader.hpp>

namespace target_system {
struct TrajectoryPoint {
  double time{};
  double pos_x, pos_y, pos_z{};
  double vel_x, vel_y, vel_z{};
};

class TrajectoryReader {
public:
  explicit TrajectoryReader();
  void read(const std::string &filename);
  TrajectoryPoint next_point();
  bool is_trajectory_finished() const;
  bool is_trajectory_read() const;

private:
  int trajectory_index_{0};
  bool is_trajectory_read_{false};
  csv2::Reader<csv2::delimiter<','>, csv2::quote_character<'"'>,
               csv2::first_row_is_header<true>,
               csv2::trim_policy::trim_whitespace>
      reader_;
  std::vector<TrajectoryPoint> trajectory_;
};
} // namespace target_system