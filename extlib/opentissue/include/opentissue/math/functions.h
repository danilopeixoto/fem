// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MATH_FUNCTIONS_H
#define OPENTISSUE_MATH_FUNCTIONS_H

#include <opentissue/configuration.h>

#include <opentissue/math/constants.h>

#include <cmath>

namespace opentissue {
    namespace math {
        template <typename T>
        inline T clamp(T const &value, T const &min_value, T const &max_value) {
            assert(min_value <= max_value || !"Maximum value cannot be "
                "less than minimum value.");
            using std::max;
            using std::min;

            return T(min(max_value, max(min_value, value)));
        }

        template <typename T> inline T clamp_min(T const &value, T const &min_value) {
            using std::max;
            return clamp(value, min_value, max(value, min_value));
        }

        template <typename T> inline T clamp_max(T const &value, T const &max_value) {
            using std::min;
            return clamp(value, min(value, max_value), max_value);
        }

        template <typename T> inline T clamp_zero_one(T const &value) {
            return clamp(value, detail::zero<T>(), detail::one<T>());
        }

        template <typename T> inline T fac(unsigned long n) {
            unsigned long val = 1;

            for (; n > 0; val *= n--);

            return T(val);
        }

        template <typename T> inline T sgn(T const &val) {
            static T const zero = detail::zero<T>();
            static T const one = detail::one<T>();

            return val > zero ? one : val < zero ? -one : zero;
        }

        template <typename T> inline T sinc(T &x) {
            using std::fabs;
            using std::sin;

            static T const tiny = static_cast<T>(1.0e-4);
            static T const factor = static_cast<T>(0.166666666666666666667);

            return (fabs(x) < tiny) ? (detail::one<T>() - x * x * factor) : (sin(x) / x);
        }

        template <typename T> inline T to_degrees(T const &radians) {
            return radians * detail::radian<T>();
        }

        template <typename T> inline T to_radians(T const &degrees) {
            return degrees * detail::degree<T>();
        }
    }
}

#endif