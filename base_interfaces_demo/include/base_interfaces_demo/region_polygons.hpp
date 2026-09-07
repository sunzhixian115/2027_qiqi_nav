// 共享区域多边形定义 — region_polygons.hpp
// 5 区域：1=基地 2=我方通道 3=高地 4=敌方通道 5=敌方基地
#ifndef BASE_INTERFACES_DEMO__REGION_POLYGONS_HPP_
#define BASE_INTERFACES_DEMO__REGION_POLYGONS_HPP_

#include <utility>
#include <vector>

namespace region_polygons {

struct Region {
  int id;
  std::vector<std::pair<double, double>> vertices;
};

inline const std::vector<Region> kRegions = {
  // 区域 1: 基地区 (HOME) — 原 0+1 合并
  {1, {{-0.605, 11.1}, {9.09, 11.2}, {9.32, 9.05}, {7.13, 6.4}, {-0.717, 6.42},
       {-3.0, 6.01}, {-0.717, 6.42}, {7.13, 6.4}, {6.85, -1.37}, {4.04, -1.68},
       {3.92, 0.188}, {0.497, 0.188}, {0.43, -3.65}, {-2.95, -3.72}}},

  // 区域 2: 我方通道区 (CORRIDOR)
  {2, {{0.497, 0.0287}, {3.92, 0.188}, {4.04, -1.68}, {6.85, -1.37},
       {10.4, -1.77}, {10.2, -3.71}, {0.43, -3.65}}},

  // 区域 3: 高地区 (HIGH) — 原 3+4 合并
  {3, {{9.32, 9.05}, {11.6, 9.28}, {11.1, 5.19}, {9.44, 3.54},
       {10.8, 2.33}, {10.4, -1.77}, {6.85, -1.37}, {7.13, 6.4},
       {11.6, 9.28}, {15.2, 8.92}, {14.8, 1.62}, {12.0, -1.73},
       {10.4, -1.77}, {10.8, 2.33}, {12.5, 3.82}, {11.1, 5.19}}},

  // 区域 4: 敌方通道区 (ENEMY_CORRIDOR)
  {4, {{11.7, 11.2}, {21.5, 11.1}, {21.4, 7.33}, {18.2, 7.27},
       {18.2, 9.04}, {15.2, 8.92}, {11.6, 9.28}}},

  // 区域 5: 敌方基地区 (ENEMY) — 原 6+7 合并
  {5, {{15.2, 8.92}, {18.2, 9.04}, {18.2, 7.27}, {21.4, 7.33},
       {21.5, 11.1}, {25.1, 11.1}, {25.0, 1.32}, {22.2, 0.8}, {14.8, 1.62},
       {22.2, 0.8}, {22.3, -3.75}, {11.9, -3.69}, {12.0, -1.73}, {14.8, 1.62}}},
};

/// 射线法判定点是否在多边形内
inline bool pointInPolygon(double px, double py,
                           const std::vector<std::pair<double, double>> & vertices) {
  const size_t n = vertices.size();
  bool inside = false;
  size_t j = n - 1;
  for (size_t i = 0; i < n; ++i) {
    const double xi = vertices[i].first;
    const double yi = vertices[i].second;
    const double xj = vertices[j].first;
    const double yj = vertices[j].second;
    if (((yi > py) != (yj > py)) &&
        (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
    j = i;
  }
  return inside;
}

}  // namespace region_polygons

#endif  // BASE_INTERFACES_DEMO__REGION_POLYGONS_HPP_
