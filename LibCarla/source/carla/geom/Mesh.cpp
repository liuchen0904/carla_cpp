// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <carla/geom/Mesh.h>

#include <string>
#include <sstream>
#include <ios>
#include <iostream>
#include <fstream>

#include <carla/geom/Math.h>

namespace carla {
namespace geom {

  // 检查 Mesh 对象是否有效
  bool Mesh::IsValid() const {
    // 检查是否至少包含一个顶点
    if (_vertices.empty()) {
      std::cout << "Mesh validation error: there are no vertices in the mesh." << std::endl;
      return false;
    }

    // 如果有索引数据，检查其数量是否是 3 的倍数（每组三个点构成一个三角形）
    if (!_indexes.empty() && _indexes.size() % 3 != 0) {
      std::cout << "Mesh validation error: the index amount must be multiple of 3." << std::endl;
      return false;
    }

    // 检查材料数据是否闭合，即最后一个材料是否被正确终止
    if (!_materials.empty() && _materials.back().index_end == 0) {
      std::cout << "Mesh validation error: last material was not closed." << std::endl;
      return false;
    }

    // 如果所有检查通过，Mesh 是有效的
    return true;
  }

  // 添加三角形条带（Triangle Strip）
  // 通过连续的顶点构建三角形，每增加一个顶点会形成一个新的三角形
  void Mesh::AddTriangleStrip(const std::vector<Mesh::vertex_type> &vertices) {
    if (vertices.size() == 0) { // 如果没有顶点，则直接返回
      return;
    }

    DEBUG_ASSERT(vertices.size() >= 3); // 调试断言：至少需要 3 个顶点
    size_t i = GetVerticesNum() + 2; // 从第三个顶点开始创建三角形
    AddVertices(vertices); // 添加所有顶点到当前的顶点列表
    bool index_clockwise = true; // 初始顺时针方向
    while (i < GetVerticesNum()) {
      index_clockwise = !index_clockwise; // 交替切换三角形的方向
      if (index_clockwise) {
        AddIndex(i + 1);
        AddIndex(i);
        AddIndex(i - 1);
      } else {
        AddIndex(i - 1);
        AddIndex(i);
        AddIndex(i + 1);
      }
      ++i; // 移动到下一个顶点
    }
  }

  // 添加三角形扇（Triangle Fan）
  // 三角形扇的第一个顶点是中心点，后续的顶点围绕中心点构建三角形
  void Mesh::AddTriangleFan(const std::vector<Mesh::vertex_type> &vertices) {
    DEBUG_ASSERT(vertices.size() >= 3); // 调试断言：至少需要 3 个顶点
    const size_t initial_index = GetVerticesNum() + 1; // 扇形的中心点索引
    size_t i = GetVerticesNum() + 2; // 从第二个顶点开始构建三角形
    AddVertices(vertices); // 添加所有顶点到当前的顶点列表
    while (i < GetVerticesNum()) {
      AddIndex(initial_index); // 中心点
      AddIndex(i);             // 当前顶点
      AddIndex(i + 1);         // 下一个顶点
      ++i; // 移动到下一个顶点
    }
  }

  // 添加一个顶点到 Mesh
  void Mesh::AddVertex(vertex_type vertex) {
    _vertices.push_back(vertex);
  }

  // 批量添加顶点到 Mesh
  void Mesh::AddVertices(const std::vector<Mesh::vertex_type> &vertices) {
    std::copy(vertices.begin(), vertices.end(), std::back_inserter(_vertices));
  }

  // 添加法线向量到 Mesh
  void Mesh::AddNormal(normal_type normal) {
    _normals.push_back(normal);
  }

  // 添加索引到 Mesh
  void Mesh::AddIndex(index_type index) {
    _indexes.push_back(index);
  }

  // 添加纹理坐标到 Mesh
  void Mesh::AddUV(uv_type uv) {
    _uvs.push_back(uv);
  }

  // 批量添加纹理坐标到 Mesh
  void Mesh::AddUVs(const std::vector<uv_type> & uv) {
    std::copy(uv.begin(), uv.end(), std::back_inserter(_uvs));
  }

  // 添加材质到 Mesh
  void Mesh::AddMaterial(const std::string &material_name) {
    const size_t open_index = _indexes.size(); // 当前索引的大小
    if (!_materials.empty()) {
      if (_materials.back().index_end == 0) { // 检查上一个材质是否被正确结束
        EndMaterial(); // 如果没有结束，则强制结束它
      }
    }
    if (open_index % 3 != 0) { // 确保索引数是 3 的倍数
      std::cout << "open_index % 3 != 0" << std::endl;
      return;
    }
    _materials.emplace_back(material_name, open_index, 0); // 添加一个新材质
  }

  // 结束当前材质的定义
  void Mesh::EndMaterial() {
    const size_t close_index = _indexes.size();
    if (_materials.empty() || // 如果没有材质
        _materials.back().index_start == close_index || // 或者材质的起始点和当前索引一致
        _materials.back().index_end != 0) { // 或者材质已经结束
      return;
    }
    if (_indexes.empty() || close_index % 3 != 0) { // 检查索引是否合法
      return;
    }
    _materials.back().index_end = close_index; // 设置材质的结束点
  }

  // 生成 OBJ 格式的字符串表示
  std::string Mesh::GenerateOBJ() const {
    if (!IsValid()) { // 如果 Mesh 无效，则返回空字符串
      return "";
    }
    std::stringstream out;
    out << std::fixed; // 避免科学计数法

    // 输出顶点
    out << "# List of geometric vertices, with (x, y, z) coordinates." << std::endl;
    for (auto &v : _vertices) {
      out << "v " << v.x << " " << v.y << " " << v.z << std::endl;
    }

    // 输出纹理坐标
    if (!_uvs.empty()) {
      out << std::endl << "# List of texture coordinates, in (u, v) coordinates." << std::endl;
      for (auto &vt : _uvs) {
        out << "vt " << vt.x << " " << vt.y << std::endl;
      }
    }

    // 输出法线
    if (!_normals.empty()) {
      out << std::endl << "# List of vertex normals in (x, y, z) form; normals might not be unit vectors." << std::endl;
      for (auto &vn : _normals) {
        out << "vn " << vn.x << " " << vn.y << " " << vn.z << std::endl;
      }
    }

    // 输出面（Polygonal faces）
    if (!_indexes.empty()) {
      out << std::endl << "# Polygonal face element." << std::endl;
      auto it_m = _materials.begin();
      auto it = _indexes.begin();
      size_t index_counter = 0u;
      while (it != _indexes.end()) {
        // 如果材质在当前索引结束，切换材质
        if (it_m != _materials.end() && it_m->index_end == index_counter) {
          ++it_m;
        }
        // 输出当前材质
        if (it_m != _materials.end() && it_m->index_start == index_counter) {
          out << "\nusemtl " << it_m->name << std::endl;
        }
        // 输出三角形面
        out << "f " << *it; ++it;
        out << " " << *it; ++it;
        out << " " << *it << std::endl; ++it;
        index_counter += 3;
      }
    }

    return out.str();
  }
} // namespace geom
} // namespace carla
