// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MATH_VECTOR_H
#define OPENTISSUE_MATH_VECTOR_H

#include <opentissue/configuration.h>

#include <opentissue/math/constants.h>
#include <opentissue/math/functions.h>
#include <opentissue/math/random.h>
#include <opentissue/math/value_traits.h>

#include <cmath>
#include <string>

#include <iostream>

namespace opentissue {
    namespace math {
        template<typename value_type_>
        class Vector {
        protected:
            typedef typename ValueTraits<value_type_> value_traits_;

        public:
            typedef value_traits_ value_traits;
            typedef value_type_ value_type;
            typedef size_t index_type;

        private:
            value_type x, y, z;

        public:
            Vector()
                : x(value_traits::zero()), y(value_traits::zero()),
                z(value_traits::zero()) {}

            Vector(Vector const &v) : x(v(0)), y(v(1)), z(v(2)) {}

            explicit Vector(value_type const &val) : x(val), y(val), z(val) {}

            template<typename T1, typename T2, typename T3>
            Vector(T1 const &x_val, T2 const &y_val, T3 const &z_val)
                : x(static_cast<value_type>(x_val)), y(static_cast<value_type>(y_val)),
                z(static_cast<value_type>(z_val)) {}

            ~Vector() {}

            Vector &operator=(Vector const &copy) {
                x = copy(0);
                y = copy(1);
                z = copy(2);

                return *this;
            }

        public:
            void clear() { x = y = z = value_traits::zero(); }

            size_t size() const { return 3u; }

        public:
            value_type &operator()(index_type index) {
                return *((&x) + index);
            }

            value_type const &operator()(index_type index) const {
                return *((&x) + index);
            }

            value_type &operator[](index_type index) {
                return *((&x) + index);
            }

            value_type const &operator[](index_type index) const {
                return *((&x) + index);
            }

        public:
            bool operator<(Vector const &v) const {
                if (x < v(0))
                    return true;

                if (x > v(0))
                    return false;

                if (y < v(1))
                    return true;

                if (y > v(1))
                    return false;

                return z < v(2);
            }

            bool operator>(Vector const &v) const {
                if (x > v(0))
                    return true;

                if (x < v(0))
                    return false;

                if (y > v(1))
                    return true;

                if (y < v(1))
                    return false;

                return z > v(2);
            }

            bool operator==(Vector const &cmp) const {
                return x == cmp.x && y == cmp.y && z == cmp.z;
            }
            bool operator!=(Vector const &cmp) const {
                return x != cmp.x || y != cmp.y || z != cmp.z;
            }

        public:
            Vector &operator+=(Vector const &v) {
                x += v(0);
                y += v(1);
                z += v(2);

                return *this;
            }

            Vector &operator-=(Vector const &v) {
                x -= v(0);
                y -= v(1);
                z -= v(2);

                return *this;
            }

            Vector &operator*=(value_type const &s) {
                x *= s;
                y *= s;
                z *= s;

                return *this;
            }

            Vector &operator/=(value_type const &s) {
                x /= s;
                y /= s;
                z /= s;

                return *this;
            }

            Vector operator+(Vector const &v) const {
                return Vector(x + v(0), y + v(1), z + v(2));
            }
            Vector operator-(Vector const &v) const {
                return Vector(x - v(0), y - v(1), z - v(2));
            }
            Vector operator-() const { return Vector(-x, -y, -z); }
            Vector operator%(Vector const &v) const {
                return Vector(y * v(2) - v(1) * z, v(0) * z - x * v(2),
                    x * v(1) - v(0) * y);
            }
            value_type operator*(Vector const &v) const {
                return x * v(0) + y * v(1) + z * v(2);
            }
            bool operator<=(Vector const &v) const {
                return x <= v(0) && y <= v(1) && z <= v(2);
            }
            bool operator>=(Vector const &v) const {
                return x >= v(0) && y >= v(1) && z >= v(2);
            }

        public:
            friend Vector fabs(Vector const &v) {
                using std::fabs;
                return Vector(static_cast<value_type>(fabs(v(0))),
                    static_cast<value_type>(fabs(v(1))),
                    static_cast<value_type>(fabs(v(2))));
            }

            friend Vector min(Vector const &A, Vector const &B) {
                using std::fmin;
                return Vector(fmin(A(0), B(0)), fmin(A(1), B(1)), fmin(A(2), B(2)));
            }

            friend Vector max(Vector const &A, Vector const &B) {
                using std::fmax;
                return Vector(fmax(A(0), B(0)), fmax(A(1), B(1)), fmax(A(2), B(2)));
            }

            friend Vector floor(Vector const &v) {
                using std::floor;
                return Vector(static_cast<value_type>(floor(v(0))),
                    static_cast<value_type>(floor(v(1))),
                    static_cast<value_type>(floor(v(2))));
            }

            friend Vector ceil(Vector const &v) {
                using std::ceil;
                return Vector(static_cast<value_type>(ceil(v(0))),
                    static_cast<value_type>(ceil(v(1))),
                    static_cast<value_type>(ceil(v(2))));
            }

            friend std::ostream &operator<<(std::ostream &o, Vector const &v) {
                o << "[";
                o << v(0);
                o << ",";
                o << v(1);
                o << "," << v(2) << "]";

                return o;
            }

            friend std::istream &operator>>(std::istream &i, Vector &v) {
                char dummy;
                i >> dummy;
                i >> v(0);
                i >> dummy;
                i >> v(1);
                i >> dummy;
                i >> v(2);
                i >> dummy;

                return i;
            }

        public:
            bool is_equal(Vector const &v, value_type const &threshold) const {
                using std::fabs;
                return fabs(x - v.x) <= threshold && fabs(y - v.y) <= threshold &&
                    fabs(z - v.z) <= threshold;
            }

        };

        template<typename T> inline Vector<T> round(Vector<T> const &v) {
            using std::floor;

            static T const half = detail::half<T>();

            return Vector<T>(floor(v(0) + half), floor(v(1) + half), floor(v(2) + half));
        }

        template<typename T>
        inline typename Vector<T>::index_type min_index(Vector<T> const &v) {
            return v(0) <= v(1) && v(0) < v(2) ? 0 : v(1) <= v(0) && v(1) < v(2) ? 1 : 2;
        }

        template<typename T>
        inline typename Vector<T>::index_type max_index(Vector<T> const &v) {
            return v(2) >= v(0) && v(2) >= v(1) ? 2 : v(1) >= v(0) && v(1) > v(2) ? 1 : 0;
        }

        template<typename T>
        inline typename Vector<T>::index_type mid_index(Vector<T> const &v) {
            typename Vector<T>::index_type test = min_index(v) + max_index(v);
            return test == 2 ? 1 : test == 1 ? 2 : 0;
        }

        template<typename T> inline T min_value(Vector<T> const &v) {
            using std::fmin;
            return min(v(0), min(v(1), v(2)));
        }

        template<typename T> inline T max_value(Vector<T> const &v) {
            using std::fmax;

            return fmax(v(0), fmax(v(1), v(2)));
        }

        template<typename T> inline T mid_value(Vector<T> const &v) {
            return v(mid_index(v));
        }

        template<typename T>
        inline bool is_zero(Vector<T> const &v, T const &threshold) {
            using std::fabs;

            return fabs(v(0)) <= threshold && fabs(v(1)) <= threshold &&
                fabs(v(2)) <= threshold;
        }

        template<typename T> inline bool is_zero(Vector<T> const &v) {
            typedef typename Vector<T>::value_traits value_traits;

            return is_zero(v, value_traits::zero());
        }

        template<typename T1, typename T2, typename T3>
        inline void random(Vector<T1> &v, T2 const &lower, T3 const &upper) {
            Random<T1> value(static_cast<T1>(lower), static_cast<T1>(upper));

            v(0) = value();
            v(1) = value();
            v(2) = value();
        }

        template<typename T>
        inline void random(Vector<T> &v, Vector<T> const &lower,
            Vector<T> const &upper) {
            typedef typename Vector<T>::value_traits value_traits;

            random(v, value_traits::zero(), value_traits::one());

            v(0) = (upper(0) - lower(0)) * v(0) + lower(0);
            v(1) = (upper(1) - lower(0)) * v(1) + lower(1);
            v(2) = (upper(2) - lower(2)) * v(2) + lower(2);
        }

        template<typename T> inline void random(Vector<T> &v) {
            typedef typename Vector<T>::value_traits value_traits;

            random(v, value_traits::zero(), value_traits::one());
        }

        template<typename T>
        inline Vector<T> cross(Vector<T> const &a, Vector<T> const &b) {
            return Vector<T>(a[1] * b[2] - b[1] * a[2], -a[0] * b[2] + b[0] * a[2],
                a[0] * b[1] - b[0] * a[1]);
        }

        template<typename T> inline T dot(Vector<T> const &a, Vector<T> const &b) {
            return a(0) * b(0) + a(1) * b(1) + a(2) * b(2);
        }

        template<typename T>
        inline T inner_prod(Vector<T> const &a, Vector<T> const &b) {
            return dot(a, b);
        }

        template<typename T> inline T length(Vector<T> const &v) {
            using std::sqrt;
            return static_cast<T>(sqrt(dot(v, v)));
        }

        template<typename T> inline T sqr_length(Vector<T> const &v) {
            return static_cast<T>(v * v);
        }

        template<typename T> inline T norm(Vector<T> const &v) { return length(v); }

        template<typename T> inline T norm_1(Vector<T> const &v) {
            using std::fabs;
            using std::fmax;

            return fmax(fabs(v(0)), fmax(fabs(v(1)), fabs(v(2))));
        }

        template<typename T>
        inline T distance(Vector<T> const &a, Vector<T> const &b) {
            return length(b - a);
        }

        template<typename T>
        inline T sqr_distance(Vector<T> const &a, Vector<T> const &b) {
            return sqr_length(b - a);
        }

        template<typename T> inline Vector<T> sign(Vector<T> const &v) {
            return Vector<T>(sign(v(0)), sign(v(1)), sign(v(2)));
        }

        template<typename T> inline Vector<T> unit(Vector<T> const &v) {
            typedef typename Vector<T>::value_traits value_traits;

            using std::sqrt;
            T const l = length(v);
            if (l <= value_traits::zero()) {
                return Vector<T>(value_traits::zero());
            }
            T const inv = value_traits::one() / l;
            return Vector<T>(inv * v(0), inv * v(1), inv * v(2));
        }

        template<typename T> inline Vector<T> normalize(Vector<T> const &v) {
            return unit(v);
        }

        template<typename T>
        inline void truncate(Vector<T> &v, T const &precision_value) {
            typedef typename Vector<T>::value_traits value_traits;

            v(0) = ((v(0) > -precision_value) && (v(0) < precision_value))
                ? value_traits::zero() : v(0);
            v(1) = ((v(1) > -precision_value) && (v(1) < precision_value))
                ? value_traits::zero() : v(1);
            v(2) = ((v(2) > -precision_value) && (v(2) < precision_value))
                ? value_traits::zero() : v(2);
        }

        template<typename T> inline void truncate(Vector<T> &v) {
            typedef typename Vector<T>::value_traits value_traits;

            truncate(v, value_traits::zero());
        }

        template<typename T> inline Vector<T> orthogonal(Vector<T> const &v) {
            typedef typename Vector<T>::value_traits value_traits;

            using std::fabs;

            Vector<T> tmp;

            if (fabs(v(1)) > fabs(v(0)))
                tmp = Vector<T>(value_traits::one(), value_traits::zero(),
                    value_traits::zero());
            else
                tmp = Vector<T>(value_traits::zero(), value_traits::zero(),
                    value_traits::one());

            return cross(v, tmp);
        }

        template<typename T, typename T2>
        inline Vector<T> operator*(Vector<T> const &v, T2 const &s) {
            return Vector<T>(v(0) * s, v(1) * s, v(2) * s);
        }

        template<typename T2, typename T>
        inline Vector<T> operator*(T2 const &s, Vector<T> const &v) {
            return Vector<T>(v(0) * s, v(1) * s, v(2) * s);
        }

        template<typename T, typename T2>
        inline Vector<T> operator/(Vector<T> const &v, T2 const &s) {
            return Vector<T>(v(0) / s, v(1) / s, v(2) / s);
        }

        template<typename vector_type>
        inline void orthonormal_vectors(vector_type &i, vector_type &j,
            vector_type const &k) {
            typedef typename vector_type::value_traits value_traits;

            using std::fabs;
            vector_type m_abs_k = fabs(k);

            if (m_abs_k(0) > m_abs_k(1)) {
                if (m_abs_k(0) > m_abs_k(2))
                    i = vector_type(value_traits::zero(), value_traits::one(),
                        value_traits::zero());
                else
                    i = vector_type(value_traits::one(), value_traits::zero(),
                        value_traits::zero());
            }
            else {
                if (m_abs_k(1) > m_abs_k(2))
                    i = vector_type(value_traits::zero(), value_traits::zero(),
                        value_traits::one());
                else
                    i = vector_type(value_traits::one(), value_traits::zero(),
                        value_traits::zero());
            }

            j = unit(cross(k, i));
            i = cross(j, k);
        }

        template<typename vector_type, typename permutation_container>
        inline void get_increasing_order(vector_type const &v,
            permutation_container &pi) {
            unsigned int n = v.size();

            for (unsigned int i = 0u; i < n; i++)
                pi[i] = i;

            for (unsigned int i = 1; i < n; i++) {
                for (unsigned int j = i; j > 0; j--) {
                    if (v(pi[j]) < v(pi[j - 1])) {
                        unsigned int tmp = pi[j - 1];

                        pi[j - 1] = pi[j];
                        pi[j] = tmp;
                    }
                }
            }
        }

        namespace detail {
            template<typename T> inline Vector<T> const &axis_x() {
                static Vector<T> const xa(one<T>(), zero<T>(), zero<T>());
                return xa;
            }

            template<typename T> inline Vector<T> const &axis_y() {
                static Vector<T> const ya(zero<T>(), one<T>(), zero<T>());
                return ya;
            }

            template<typename T> inline Vector<T> const &axis_z() {
                static Vector<T> const za(zero<T>(), zero<T>(), one<T>());
                return za;
            }
        }
    }
}

#endif