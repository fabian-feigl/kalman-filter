#include <exception>
#include <iostream>
#include <string>
#include <vector>

template <typename T> class Matrix {
private:
  std::vector<T> inner_;
  unsigned int dimx_, dimy_;

public:
  Matrix(unsigned int dimx, unsigned int dimy) : dimx_(dimx), dimy_(dimy) {
    inner_.resize(dimx_ * dimy_);
  }

  T &operator()(unsigned int x, unsigned int y) {
    if (x >= dimx_ || y >= dimy_)
      throw std::out_of_range("matrix indices out of range"); // ouch
    return inner_[dimx_ * y + x];
  }

  Matrix<T> operator+(T value) {
    for (int i = 0; i < dimx_; i++) {
      for (int j = 0; j < dimy_; j++) {
        (*this)(i, j) += value;
      }
    }
    return (*this);
  }

  Matrix<T> operator+(Matrix<T> other) {
    for (int i = 0; i < dimx_; i++) {
      for (int j = 0; j < dimy_; j++) {
        (*this)(i, j) += other(i, j);
      }
    }
    return (*this);
  }

  Matrix<T> operator*(T value) {
    for (int i = 0; i < dimx_; i++) {
      for (int j = 0; j < dimy_; j++) {
        (*this)(i, j) = (*this)(i, j) * value;
      }
    }
    return (*this);
  }

  Matrix<T> operator*(Matrix<T> other) {
    if (other.dimx_ != dimy_)
      throw std::out_of_range(
          "matrices cannot be multiplied. This x=" + std::to_string(dimx_) +
          " and y=" + std::to_string(dimy_) +
          ", other x=" + std::to_string(other.dimx_) +
          " and y=" + std::to_string(other.dimy_));

    Matrix<T> matrix_out(dimx_, other.dimy_);
    for (int i = 0; i < matrix_out.dimx_; i++) {
      for (int j = 0; j < matrix_out.dimy_; j++) {
        for (int k = 0; k < dimy_; k++) {
          matrix_out(i, j) += (*this)(i, k) * other(k, j);
        }
      }
    }
    return matrix_out;
  }

  std::vector<T> operator*(std::vector<T> other) {
    if (other.size() != dimx_)
      throw std::out_of_range("vector cannot be multiplied"); // ouch

    std::vector<T> vector_out(dimx_);
    for (int i = 0; i < vector_out.size(); i++) {
      for (int k = 0; k < dimy_; k++) {
        vector_out[i] += (*this)(i, k) * other[i];
      }
    }
    return vector_out;
  }

  Matrix<T> transpose() {
    Matrix<T> matrix_out(dimy_, dimx_);
    for (int i = 0; i < matrix_out.dimx_; i++) {
      for (int j = 0; j < matrix_out.dimy_; j++) {
        matrix_out(j, i) = (*this)(i, j);
      }
    }
    return matrix_out;
  }

  std::string print() {
    std::string out;
    for (int i = 0; i < dimx_; i++) {
      for (int j = 0; j < dimy_; j++) {
        out += std::to_string((*this)(i, j)) + " ";
      }
      out += "\n";
    }
    return out;
  }
};