#include <string>
#include <vector>

#include <csv2/reader.hpp>

namespace target_system {
struct TrajectoryPoint {
  double time{};
  double pos_x, pos_y, pos_z{};
};

class TrajectoryReader {
public:
  explicit TrajectoryReader();
  std::vector<TrajectoryPoint> read(const std::string &filename);

private:
  csv2::Reader<csv2::delimiter<','>, csv2::quote_character<'"'>,
               csv2::first_row_is_header<true>,
               csv2::trim_policy::trim_whitespace>
      reader_;
};
} // namespace target_system