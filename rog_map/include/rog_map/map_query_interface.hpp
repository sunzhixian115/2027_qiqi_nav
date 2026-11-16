#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <vector>

namespace rog_map {

enum class QueryStatus : uint8_t
{
  OK = 0,//成功
  OUT_OF_MAP,//超出地图范围
  SNAPSHOT_INVALID,//快照无效
  FIELD_UNINITIALIZED,//字段未初始化
  INTERPOLATION_FAILED,//插值失败
  TF_FAILED,//TF失败
  NONFINITE_INPUT,//非有限输入
  NONFINITE_OUTPUT,//非有限输出
};

struct QueryResult//查询结果结构体
{
  bool ok{false};
  QueryStatus status{QueryStatus::SNAPSHOT_INVALID};
  double distance{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
  uint64_t projection_sequence{0};
  uint64_t mask_sequence{0};
  uint64_t field_sequence{0};
  uint64_t snapshot_sequence{0};
  double field_age_ms{std::numeric_limits<double>::quiet_NaN()};
};

const char * queryStatusName(QueryStatus status);//获取查询状态名称

class MapQueryInterface//地图查询接口类
{
public:
  virtual ~MapQueryInterface() = default;//默认析构函数

  virtual bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const = 0;//世界坐标到地图坐标
  virtual void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const = 0;//地图坐标到世界坐标

  virtual unsigned int sizeX() const = 0;//地图宽度
  virtual unsigned int sizeY() const = 0;//地图高度
  virtual double resolution() const = 0;//地图分辨率
  virtual double originX() const = 0;//地图原点x坐标
  virtual double originY() const = 0;//地图原点y坐标

  virtual uint8_t value(unsigned int mx, unsigned int my) const = 0;//获取地图值
  virtual const unsigned char * values() const = 0;//获取地图值指针
  virtual bool copyValues(std::vector<unsigned char> & out) const//复制地图值到向量
  {
    const auto * data = values();
    const size_t count = static_cast<size_t>(sizeX()) * static_cast<size_t>(sizeY());
    if (!data || count == 0U) {
      out.clear();
      return false;
    }
    out.assign(data, data + count);//复制地图值到向量
    return true;
  }

  virtual bool isValid(unsigned int mx, unsigned int my) const = 0;//判断地图坐标是否有效
  virtual bool isFree(unsigned int mx, unsigned int my) const = 0;//判断地图坐标是否为空

  virtual QueryResult query(const Eigen::Vector3d & pos) const;//查询地图
  virtual bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const = 0;//评估地图
};

}  // namespace rog_map
