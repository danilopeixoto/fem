// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MATH_CONSTANTS_H
#define OPENTISSUE_MATH_CONSTANTS_H

#include <opentissue/configuration.h>

#include <cmath>
#include <limits>

namespace opentissue {
    namespace math {
        namespace detail {
            template <typename T> inline T zero();
            template <> inline float zero<float>() { return 0.0f; }
            template <> inline double zero<double>() { return 0.0; }
            template <> inline int zero<int>() { return 0; }
            template <> inline long unsigned int zero<long unsigned int>() { return 0; }
            template <> inline unsigned int zero<unsigned int>() { return 0u; }
        }

        namespace detail {
            template <typename T> inline T one();
            template <> inline float one<float>() { return 1.0f; }
            template <> inline double one<double>() { return 1.0; }
            template <> inline int one<int>() { return 1; }
            template <> inline unsigned int one<unsigned int>() { return 1u; }
        }

        namespace detail {
            template <typename T> inline T two();
            template <> inline float two<float>() { return 2.0f; }
            template <> inline double two<double>() { return 2.0; }
            template <> inline int two<int>() { return 2; }
            template <> inline unsigned int two<unsigned int>() { return 2u; }
        }

        namespace detail {
            template <typename T> inline T three();
            template <> inline float three<float>() { return 3.0f; }
            template <> inline double three<double>() { return 3.0; }
            template <> inline int three<int>() { return 3; }
            template <> inline unsigned int three<unsigned int>() { return 3u; }
        }

        namespace detail {
            template <typename T> inline T four();
            template <> inline float four<float>() { return 4.0f; }
            template <> inline double four<double>() { return 4.0; }
            template <> inline int four<int>() { return 4; }
            template <> inline unsigned int four<unsigned int>() { return 4u; }
        }

        namespace detail {
            template <typename T> inline T eight();
            template <> inline float eight<float>() { return 8.0f; }
            template <> inline double eight<double>() { return 8.0; }
            template <> inline int eight<int>() { return 8; }
            template <> inline unsigned int eight<unsigned int>() { return 8u; }
        }

        namespace detail {
            template <typename T> inline T half();
            template <> inline float half<float>() { return 0.5f; }
            template <> inline double half<double>() { return 0.5; }
        }

        namespace detail {
            template <typename T> inline T pi() { return static_cast<T>(M_PI); }
            template <typename T> inline T pi_half() { return static_cast<T>(M_PI_2); }
            template <typename T> inline T pi_quarter() { return static_cast<T>(M_PI_4); }
        }

        namespace detail {
            template <typename T> inline T highest() {
                return std::numeric_limits<T>::max();
            }

            template <typename T> inline T lowest() {
                return std::numeric_limits<T>::lowest();
            }

            template <typename T> inline T infinity() {
                return highest<T>();
            }

            template <typename T> inline T degree() {
                return static_cast<T>(0.017453292519943295769236907684886);
            }

            template <typename T> inline T radian() {
                return static_cast<T>(57.295779513082320876798154814105);
            }
        }
    }
}

#endif