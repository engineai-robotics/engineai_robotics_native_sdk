#pragma once

#include <iostream>
#include <vector>

template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& vec) {
  out << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    out << vec[i];
    if (i != vec.size() - 1) {
      out << ", ";
    }
  }
  out << "]";
  return out;
}

template <typename T, size_t N>
std::ostream& operator<<(std::ostream& out, const std::array<T, N>& arr) {
  out << "[";
  for (size_t i = 0; i < N; ++i) {
    out << arr[i];
    if (i != N - 1) {
      out << ", ";
    }
  }
  out << "]";
  return out;
}

namespace common {

template <typename T>
std::vector<double>& ToVectorDouble(const std::vector<Eigen::Vector3<T>>& in, std::vector<double>& out) {
  out.clear();
  out.resize(in.size() * 3);
  for (int i = 0; i < in.size(); ++i) {
    out.at(i * 3) = in[i].x();
    out.at(i * 3 + 1) = in[i].y();
    out.at(i * 3 + 2) = in[i].z();
  }

  return out;
}

template <typename T, int Rows>
std::vector<double>& ToVectorDouble(const Eigen::Matrix<T, Rows, 1>& in, std::vector<double>& out) {
  out.clear();
  out = std::vector<double>(in.data(), in.data() + in.size());
  return out;
}

template <typename T, size_t N>
std::vector<double>& ToVectorDouble(const std::array<T, N>& in, std::vector<double>& out) {
  out.clear();
  out = std::vector<double>(in.begin(), in.end());
  return out;
}
}  // namespace common
