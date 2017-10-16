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
        template<typename T>
        inline T clamp(T const &value, T const &min_value, T const &max_value) {
            using std::fmax;
            using std::fmin;

            return static_cast<T>(fmin(max_value, fmax(min_value, value)));
        }

        template<typename T> inline T fac(unsigned long n) {
            unsigned long value = 1;

            for (; n > 0; value *= n--);

            return static_cast<T>(value);
        }

        template<typename T> inline T sign(T const &value) {
            T const zero = detail::zero<T>();
            T const one = detail::one<T>();

            if (value > zero)
                return one;
            else if (value < zero)
                return -one;

            return zero;
        }

        template<typename T> inline T to_degrees(T const &radians) {
            return radians * detail::radian<T>();
        }

        template<typename T> inline T to_radians(T const &degrees) {
            return degrees * detail::degree<T>();
        }
    }
}

#endif