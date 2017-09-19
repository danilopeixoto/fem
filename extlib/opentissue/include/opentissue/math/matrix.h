// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MATH_MATRIX_H
#define OPENTISSUE_MATH_MATRIX_H

#include <opentissue/configuration.h>

#include <opentissue/math/eigen_system_decomposition.h>
#include <opentissue/math/value_traits.h>
#include <opentissue/math/vector.h>

#include <cassert>
#include <cmath>
#include <iosfwd>

namespace opentissue {
    namespace math {

        template <typename> class Quaternion;

        template <typename value_type_>
        class Matrix {
        protected:
            typedef typename math::ValueTraits<value_type_> value_traits_;

        public:
            typedef value_traits_ value_traits;
            typedef value_type_ value_type;

            typedef Vector<value_type> vector_type;
            typedef Quaternion<value_type> quaternion_type;
            typedef size_t index_type;

        protected:

            vector_type m_row0;
            vector_type m_row1;
            vector_type m_row2;

        public:
            Matrix()
                : m_row0(value_traits::zero(), value_traits::zero(),
                    value_traits::zero()),
                m_row1(value_traits::zero(), value_traits::zero(),
                    value_traits::zero()),
                m_row2(value_traits::zero(), value_traits::zero(),
                    value_traits::zero()) {}

            ~Matrix() {}

            explicit Matrix(value_type const &m00, value_type const &m01,
                value_type const &m02, value_type const &m10,
                value_type const &m11, value_type const &m12,
                value_type const &m20, value_type const &m21,
                value_type const &m22)
                : m_row0(m00, m01, m02), m_row1(m10, m11, m12), m_row2(m20, m21, m22) {}

            explicit Matrix(vector_type const &row0, vector_type const &row1,
                vector_type const &row2)
                : m_row0(row0), m_row1(row1), m_row2(row2) {}

            explicit Matrix(quaternion_type const &q) { *this = q; }

            Matrix(Matrix const &M)
                : m_row0(M.m_row0), m_row1(M.m_row1), m_row2(M.m_row2) {}

        public:
            value_type &operator()(index_type i, index_type j) {
                assert((i >= 0 && i < 3) && (j >= 0 && j < 3) ||
                    !"Matrix index out of range.");
                return (*(&m_row0 + i))(j);
            }

            value_type const &operator()(index_type i, index_type j) const {
                assert((i >= 0 && i < 3) && (j >= 0 && j < 3) ||
                    !"Matrix index out of range.");
                return (*(&m_row0 + i))(j);
            }

            vector_type &operator[](index_type i) {
                assert((i >= 0 && i < 3) || !"Matrix index out of range.");
                return *(&m_row0 + i);
            }

            vector_type const &operator[](index_type i) const {
                assert((i >= 0 && i < 3) || !"Matrix index out of range.");
                return *(&m_row0 + i);
            }

            Matrix &operator=(Matrix const &cpy) {
                m_row0 = cpy.m_row0;
                m_row1 = cpy.m_row1;
                m_row2 = cpy.m_row2;
                return *this;
            }

            bool operator==(Matrix const &cmp) const {
                return (m_row0 == cmp.m_row0) && (m_row1 == cmp.m_row1) &&
                    (m_row2 == cmp.m_row2);
            }

            bool operator!=(Matrix const &cmp) const { return !(*this == cmp); }

            Matrix operator+(Matrix const &m) const {
                return Matrix(m_row0 + m.m_row0, m_row1 + m.m_row1, m_row2 + m.m_row2);
            }
            Matrix operator-(Matrix const &m) const {
                return Matrix(m_row0 - m.m_row0, m_row1 - m.m_row1, m_row2 - m.m_row2);
            }
            Matrix &operator+=(Matrix const &m) {
                m_row0 += m.m_row0;
                m_row1 += m.m_row1;
                m_row2 += m.m_row2;
                return *this;
            }
            Matrix &operator-=(Matrix const &m) {
                m_row0 -= m.m_row0;
                m_row1 -= m.m_row1;
                m_row2 -= m.m_row2;
                return *this;
            }
            Matrix &operator*=(value_type const &s) {
                m_row0 *= s;
                m_row1 *= s;
                m_row2 *= s;
                return *this;
            }

            Matrix &operator/=(value_type const &s) {
                assert(s || !"Division by zero.");
                m_row0 /= s;
                m_row1 /= s;
                m_row2 /= s;
                return *this;
            }

            vector_type operator*(vector_type const &v) const {
                return vector_type(m_row0 * v, m_row1 * v, m_row2 * v);
            }
            Matrix operator-() const { return Matrix(-m_row0, -m_row1, -m_row2); }

            size_t size1() const { return 3u; }
            size_t size2() const { return 3u; }

            Matrix &operator=(quaternion_type const &q) {
                m_row0(0) =
                    value_traits::one() -
                    value_traits::two() * ((q.v()(1) * q.v()(1)) + (q.v()(2) * q.v()(2)));
                m_row1(1) =
                    value_traits::one() -
                    value_traits::two() * ((q.v()(0) * q.v()(0)) + (q.v()(2) * q.v()(2)));
                m_row2(2) =
                    value_traits::one() -
                    value_traits::two() * ((q.v()(1) * q.v()(1)) + (q.v()(0) * q.v()(0)));
                m_row1(0) =
                    value_traits::two() * ((q.v()(0) * q.v()(1)) + (q.s() * q.v()(2)));
                m_row0(1) =
                    value_traits::two() * ((q.v()(0) * q.v()(1)) - (q.s() * q.v()(2)));
                m_row2(0) =
                    value_traits::two() * (-(q.s() * q.v()(1)) + (q.v()(0) * q.v()(2)));
                m_row0(2) =
                    value_traits::two() * ((q.s() * q.v()(1)) + (q.v()(0) * q.v()(2)));
                m_row2(1) =
                    value_traits::two() * ((q.v()(2) * q.v()(1)) + (q.s() * q.v()(0)));
                m_row1(2) =
                    value_traits::two() * ((q.v()(2) * q.v()(1)) - (q.s() * q.v()(0)));
                return *this;
            }

            void clear() {
                m_row0.clear();
                m_row1.clear();
                m_row2.clear();
            }

        public:
            friend Matrix fabs(Matrix const &A) {
                using std::fabs;

                return Matrix(fabs(A(0, 0)), fabs(A(0, 1)), fabs(A(0, 2)), fabs(A(1, 0)),
                    fabs(A(1, 1)), fabs(A(1, 2)), fabs(A(2, 0)), fabs(A(2, 1)),
                    fabs(A(2, 2)));
            }

        public:
            vector_type column(index_type i) const {
                assert((i >= 0 && i < 3) || !"Matrix index out of range.");

                return vector_type(m_row0(i), m_row1(i), m_row2(i));
            }

            void set_column(index_type i, vector_type const &column) {
                assert((i >= 0 && i < 3) || !"Matrix index out of range.");

                m_row0(i) = column(0);
                m_row1(i) = column(1);
                m_row2(i) = column(2);
            }

            vector_type &row(index_type i) { return (*this)[i]; }
            vector_type const &row(index_type i) const { return (*this)[i]; }
        };

        template <typename T> inline void random(Matrix<T> &m) {
            random(m.row(0));
            random(m.row(1));
            random(m.row(2));
        }

        template <typename T>
        inline Matrix<T> operator*(Matrix<T> const &m, T const &s) {
            return Matrix<T>(m.row(0) * s, m.row(1) * s, m.row(2) * s);
        }

        template <typename T>
        inline Matrix<T> operator*(T const &s, Matrix<T> const &m) {
            return Matrix<T>(m.row(0) * s, m.row(1) * s, m.row(2) * s);
        }

        template <typename T>
        inline Matrix<T> operator/(Matrix<T> const &m, T const &s) {
            return Matrix<T>(m.row(0) / s, m.row(1) / s, m.row(2) / s);
        }

        template <typename T>
        inline Matrix<T> operator/(T const &s, Matrix<T> const &m) {
            return Matrix<T>(m.row(0) / s, m.row(1) / s, m.row(2) / s);
        }

        template <typename T>
        inline Matrix<T> operator*(Matrix<T> const &A, Matrix<T> const &B) {
            typedef typename Matrix<T>::value_traits value_traits;
            Matrix<T> C;

            for (int c = 0; c < 3; ++c)
                for (int r = 0; r < 3; ++r) {
                    C(r, c) = value_traits::zero();
                    for (int i = 0; i < 3; ++i) {
                        C(r, c) += A(r, i) * B(i, c);
                    }
                }
            return C;
        }

        template <typename T> inline Matrix<T> Rx(T const &radians) {
            typedef typename Matrix<T>::value_traits value_traits;

            using std::cos;
            using std::sin;

            T cosinus = static_cast<T>(cos(radians));
            T sinus = static_cast<T>(sin(radians));

            return Matrix<T>(value_traits::one(), value_traits::zero(),
                value_traits::zero(), value_traits::zero(), cosinus,
                -sinus, value_traits::zero(), sinus, cosinus);
        }

        template <typename T> inline Matrix<T> Ry(T const &radians) {
            typedef typename Matrix<T>::value_traits value_traits;

            using std::cos;
            using std::sin;

            T cosinus = static_cast<T>(cos(radians));
            T sinus = static_cast<T>(sin(radians));
            return Matrix<T>(cosinus, value_traits::zero(), sinus,
                value_traits::zero(), value_traits::one(),
                value_traits::zero(), -sinus, value_traits::zero(),
                cosinus);
        }

        template <typename T> inline Matrix<T> Rz(T const &radians) {
            typedef typename Matrix<T>::value_traits value_traits;

            using std::cos;
            using std::sin;

            T cosinus = static_cast<T>(cos(radians));
            T sinus = static_cast<T>(sin(radians));

            return Matrix<T>(cosinus, -sinus, value_traits::zero(), sinus, cosinus,
                value_traits::zero(), value_traits::zero(),
                value_traits::zero(), value_traits::one());
        }

        template <typename T>
        inline Matrix<T> Ru(T const &radians, Vector<T> const &axis) {
            typedef typename Matrix<T>::value_traits value_traits;

            using std::cos;
            using std::sin;

            T cosinus = static_cast<T>(cos(radians));
            T sinus = static_cast<T>(sin(radians));
            Vector<T> u = unit(axis);

            return Matrix<T>(
                u(0) * u(0) + cosinus * (value_traits::one() - u(0) * u(0)),
                u(0) * u(1) * (value_traits::one() - cosinus) - sinus * u(2),
                u(0) * u(2) * (value_traits::one() - cosinus) + sinus * u(1),
                u(0) * u(1) * (value_traits::one() - cosinus) + sinus * u(2),
                u(1) * u(1) + cosinus * (value_traits::one() - u(1) * u(1)),
                u(1) * u(2) * (value_traits::one() - cosinus) - sinus * u(0),
                u(0) * u(2) * (value_traits::one() - cosinus) - sinus * u(1),
                u(1) * u(2) * (value_traits::one() - cosinus) + sinus * u(0),
                u(2) * u(2) + cosinus * (value_traits::one() - u(2) * u(2)));
        }

        template <typename T> inline Matrix<T> z_dof(Vector<T> const &k) {
            Vector<T> i, j;
            orthonormal_vectors(i, j, unit(k));
            return Matrix<T>(i(0), j(0), k(0), i(1), j(1), k(1), i(2), j(2), k(2));
        }

        template <typename T>
        inline bool is_orthonormal(Matrix<T> const &M, T const &threshold) {
            typedef typename Matrix<T>::value_traits value_traits;

            using std::fabs;

            assert(threshold >= value_traits::zero() ||
                !"Threshold must be non-negative.");

            T dot01 = M.m_row0 * M.m_row1;
            T dot02 = M.m_row0 * M.m_row2;
            T dot12 = M.m_row1 * M.m_row2;
            if (fabs(dot01) > threshold)
                return false;
            if (fabs(dot02) > threshold)
                return false;
            if (fabs(dot12) > threshold)
                return false;
            T dot00 = M.m_row0 * M.m_row0;
            T dot11 = M.m_row1 * M.m_row1;
            T dot22 = M.m_row2 * M.m_row2;
            if ((dot00 - value_traits::one()) > threshold)
                return false;
            if ((dot11 - value_traits::one()) > threshold)
                return false;
            if ((dot22 - value_traits::one()) > threshold)
                return false;
            return true;
        }

        template <typename T> inline bool is_orthonormal(Matrix<T> const &M) {
            typedef typename Matrix<T>::value_traits value_traits;

            return is_orthonormal(M, value_traits::zero());
        }

        template <typename T> inline bool is_zero(Matrix<T> M, T const &threshold) {
            typedef typename Matrix<T>::value_traits value_traits;

            using std::fabs;

            assert(threshold >= value_traits::zero() ||
                !"Threshold must be non-negative.");

            if (fabs(M(0, 0)) > threshold)
                return false;
            if (fabs(M(0, 1)) > threshold)
                return false;
            if (fabs(M(0, 2)) > threshold)
                return false;
            if (fabs(M(1, 0)) > threshold)
                return false;
            if (fabs(M(1, 1)) > threshold)
                return false;
            if (fabs(M(1, 2)) > threshold)
                return false;
            if (fabs(M(2, 0)) > threshold)
                return false;
            if (fabs(M(2, 1)) > threshold)
                return false;
            if (fabs(M(2, 2)) > threshold)
                return false;
            return true;
        }

        template <typename T> inline bool is_zero(Matrix<T> const &M) {
            typedef typename Matrix<T>::value_traits value_traits;

            return is_zero(M, value_traits::zero());
        }

        template <typename T>
        inline bool is_symmetric(Matrix<T> M, T const &threshold) {
            typedef typename Matrix<T>::value_traits value_traits;

            using std::fabs;

            assert(threshold >= value_traits::zero() ||
                !"Threshold must be non-negative.");

            if (fabs(M(0, 1) - M(1, 0)) > threshold)
                return false;
            if (fabs(M(0, 2) - M(2, 0)) > threshold)
                return false;
            if (fabs(M(1, 2) - M(2, 1)) > threshold)
                return false;
            return true;
        }

        template <typename T> inline bool is_symmetric(Matrix<T> const &M) {
            typedef typename Matrix<T>::value_traits value_traits;

            return is_symmetric(M, value_traits::zero());
        }

        template <typename T>
        inline bool is_diagonal(Matrix<T> M, T const &threshold) {
            typedef typename Matrix<T>::value_traits value_traits;

            using std::fabs;

            assert(threshold >= value_traits::zero() ||
                !"Threshold must be non-negative.");

            if (fabs(M(0, 1)) > threshold)
                return false;
            if (fabs(M(0, 2)) > threshold)
                return false;
            if (fabs(M(1, 0)) > threshold)
                return false;
            if (fabs(M(1, 2)) > threshold)
                return false;
            if (fabs(M(2, 0)) > threshold)
                return false;
            if (fabs(M(2, 1)) > threshold)
                return false;
            return true;
        }

        template <typename T> inline bool is_diagonal(Matrix<T> const &M) {
            typedef typename Matrix<T>::value_traits value_traits;

            return is_diagonal(M, value_traits::zero());
        }

        template <typename T>
        inline bool is_identity(Matrix<T> M, T const &threshold) {
            typedef typename Matrix<T>::value_traits value_traits;

            using std::fabs;

            assert(threshold >= value_traits::zero() ||
                !"Threshold must be non-negative.");

            if (fabs(M(0, 0) - value_traits::one()) > threshold)
                return false;
            if (fabs(M(0, 1)) > threshold)
                return false;
            if (fabs(M(0, 2)) > threshold)
                return false;
            if (fabs(M(1, 0)) > threshold)
                return false;
            if (fabs(M(1, 1) - value_traits::one()) > threshold)
                return false;
            if (fabs(M(1, 2)) > threshold)
                return false;
            if (fabs(M(2, 0)) > threshold)
                return false;
            if (fabs(M(2, 1)) > threshold)
                return false;
            if (fabs(M(2, 2) - value_traits::one()) > threshold)
                return false;

            return true;
        }

        template <typename T> inline bool is_identity(Matrix<T> const &M) {
            typedef typename Matrix<T>::value_traits value_traits;

            return is_identity(M, value_traits::zero());
        }

        template <typename T>
        inline Matrix<T> outer_prod(Vector<T> const &v1, Vector<T> const &v2) {
            return Matrix<T>((v1(0) * v2(0)), (v1(0) * v2(1)), (v1(0) * v2(2)),
                (v1(1) * v2(0)), (v1(1) * v2(1)), (v1(1) * v2(2)),
                (v1(2) * v2(0)), (v1(2) * v2(1)), (v1(2) * v2(2)));
        }

        template <typename T> inline Matrix<T> ortonormalize(Matrix<T> const &A) {
            typedef typename Matrix<T>::vector_type vector_type;

            vector_type row0(A(0, 0), A(0, 1), A(0, 2));
            vector_type row1(A(1, 0), A(1, 1), A(1, 2));
            vector_type row2(A(2, 0), A(2, 1), A(2, 2));

            T const l0 = length(row0);
            if (l0)
                row0 /= l0;

            row1 -= row0 * dot(row0, row1);
            T const l1 = length(row1);
            if (l1)
                row1 /= l1;

            row2 = cross(row0, row1);

            return Matrix<T>(row0(0), row0(1), row0(2), row1(0), row1(1), row1(2),
                row2(0), row2(1), row2(2));
        }

        template <typename T> inline Matrix<T> trans(Matrix<T> const &M) {
            return Matrix<T>(M(0, 0), M(1, 0), M(2, 0), M(0, 1), M(1, 1), M(2, 1),
                M(0, 2), M(1, 2), M(2, 2));
        }

        template <typename T>
        inline Matrix<T> diag(T const &d0, T const &d1, T const &d2) {
            typedef typename Matrix<T>::value_traits value_traits;

            return Matrix<T>(d0, value_traits::zero(), value_traits::zero(),
                value_traits::zero(), d1, value_traits::zero(),
                value_traits::zero(), value_traits::zero(), d2);
        }

        template <typename T> inline Matrix<T> diag(Vector<T> const &d) {
            return diag(d(0), d(1), d(2));
        }

        template <typename T> inline Matrix<T> diag(T const &d) {
            return diag(d, d, d);
        }

        template <typename T> inline Matrix<T> inverse(Matrix<T> const &A) {
            typedef typename Matrix<T>::value_traits value_traits;

            Matrix<T> adj;
            adj(0, 0) = A(1, 1) * A(2, 2) - A(2, 1) * A(1, 2);
            adj(1, 1) = A(0, 0) * A(2, 2) - A(2, 0) * A(0, 2);
            adj(2, 2) = A(0, 0) * A(1, 1) - A(1, 0) * A(0, 1);
            adj(0, 1) = A(1, 0) * A(2, 2) - A(2, 0) * A(1, 2);
            adj(0, 2) = A(1, 0) * A(2, 1) - A(2, 0) * A(1, 1);
            adj(1, 0) = A(0, 1) * A(2, 2) - A(2, 1) * A(0, 2);
            adj(1, 2) = A(0, 0) * A(2, 1) - A(2, 0) * A(0, 1);
            adj(2, 0) = A(0, 1) * A(1, 2) - A(1, 1) * A(0, 2);
            adj(2, 1) = A(0, 0) * A(1, 2) - A(1, 0) * A(0, 2);

            T det = A(0, 0) * adj(0, 0) - A(0, 1) * adj(0, 1) + A(0, 2) * adj(0, 2);

            if (det) {
                adj(0, 1) = -adj(0, 1);
                adj(1, 0) = -adj(1, 0);
                adj(1, 2) = -adj(1, 2);
                adj(2, 1) = -adj(2, 1);
                return trans(adj) / det;
            }

            return diag(value_traits::one());
        }

        template <typename T> inline T max_value(Matrix<T> const &A) {
            using std::max;

            return max(
                A(0, 0),
                max(A(0, 1),
                    max(A(0, 2),
                        max(A(1, 0),
                            max(A(1, 1),
                                max(A(1, 2), max(A(2, 0), max(A(2, 1), A(2, 2)))))))));
        }

        template <typename T> inline T min_value(Matrix<T> const &A) {
            using std::min;

            return min(
                A(0, 0),
                min(A(0, 1),
                    min(A(0, 2),
                        min(A(1, 0),
                            min(A(1, 1),
                                min(A(1, 2), min(A(2, 0), min(A(2, 1), A(2, 2)))))))));
        }

        template <typename T> inline T det(Matrix<T> const &A) {
            return A(0, 0) * (A(1, 1) * A(2, 2) - A(2, 1) * A(1, 2)) -
                A(0, 1) * (A(1, 0) * A(2, 2) - A(2, 0) * A(1, 2)) +
                A(0, 2) * (A(1, 0) * A(2, 1) - A(2, 0) * A(1, 1));
        }

        template <typename T> inline T trace(Matrix<T> const &A) {
            return (A(0, 0) + A(1, 1) + A(2, 2));
        }

        template <typename T> inline T norm_1(Matrix<T> const &A) {
            using std::fabs;
            using std::max;

            T r0 = fabs(A(0, 0)) + fabs(A(0, 1)) + fabs(A(0, 2));
            T r1 = fabs(A(1, 0)) + fabs(A(1, 1)) + fabs(A(1, 2));
            T r2 = fabs(A(2, 0)) + fabs(A(2, 1)) + fabs(A(2, 2));

            return max(r0, max(r1, r2));
        }

        template <typename T> inline T norm_2(Matrix<T> const &A) {
            using std::fabs;
            using std::max;
            using std::sqrt;

            Matrix<T> V;
            typename Matrix<T>::vector_type d;
            math::eigen(A, V, d);

            T lambda = max(fabs(d(0)), max(fabs(d(1)), fabs(d(2))));
            return sqrt(lambda);
        }

        template <typename T> inline T norm_inf(Matrix<T> const &A) {
            using std::fabs;
            using std::max;

            T c0 = fabs(A(0, 0)) + fabs(A(1, 0)) + fabs(A(2, 0));
            T c1 = fabs(A(0, 1)) + fabs(A(1, 1)) + fabs(A(2, 1));
            T c2 = fabs(A(0, 2)) + fabs(A(1, 2)) + fabs(A(2, 2));
            return max(c0, max(c1, c2));
        }

        template <typename T>
        inline Matrix<T> truncate(Matrix<T> const &A, T const &epsilon) {
            typedef typename Matrix<T>::value_traits value_traits;

            using std::fabs;

            assert(epsilon > value_traits::zero() || !"Epsilon must be positive.");

            return Matrix<T>(fabs(A(0, 0)) < epsilon ? value_traits::zero() : A(0, 0),
                fabs(A(0, 1)) < epsilon ? value_traits::zero() : A(0, 1),
                fabs(A(0, 2)) < epsilon ? value_traits::zero() : A(0, 2),

                fabs(A(1, 0)) < epsilon ? value_traits::zero() : A(1, 0),
                fabs(A(1, 1)) < epsilon ? value_traits::zero() : A(1, 1),
                fabs(A(1, 2)) < epsilon ? value_traits::zero() : A(1, 2),

                fabs(A(2, 0)) < epsilon ? value_traits::zero() : A(2, 0),
                fabs(A(2, 1)) < epsilon ? value_traits::zero() : A(2, 1),
                fabs(A(2, 2)) < epsilon ? value_traits::zero() : A(2, 2));
        }

        template <typename T> inline Matrix<T> star(Vector<T> const &v) {
            typedef typename Matrix<T>::value_traits value_traits;

            return Matrix<T>(value_traits::zero(), -v(2), v(1), v(2),
                value_traits::zero(), -v(0), -v(1), v(0),
                value_traits::zero());
        }

        template <typename T>
        inline std::ostream &operator<<(std::ostream &o, Matrix<T> const &A) {
            o << "[" << A(0, 0) << "," << A(0, 1) << "," << A(0, 2) << ";" << A(1, 0)
                << "," << A(1, 1) << "," << A(1, 2) << ";" << A(2, 0) << "," << A(2, 1)
                << "," << A(2, 2) << "]";
            return o;
        }

        template <typename T>
        inline std::istream &operator>>(std::istream &i, Matrix<T> &A) {
            char dummy;

            i >> dummy;
            i >> A(0, 0);
            i >> dummy;
            i >> A(0, 1);
            i >> dummy;
            i >> A(0, 2);
            i >> dummy;
            i >> A(1, 0);
            i >> dummy;
            i >> A(1, 1);
            i >> dummy;
            i >> A(1, 2);
            i >> dummy;
            i >> A(2, 0);
            i >> dummy;
            i >> A(2, 1);
            i >> dummy;
            i >> A(2, 2);
            i >> dummy;

            return i;
        }
    }
}

#endif