// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MATH_RANDOM_H
#define OPENTISSUE_MATH_RANDOM_H

#include <opentissue/configuration.h>

#include <opentissue/math/constants.h>

#include <cstdlib>
#include <ctime>

namespace opentissue {
    namespace math {
        template<typename value_type> class Random {
        protected:
            typedef value_type T;
            typedef Random<T> self;

            T m_lower;
            T m_upper;

            static bool &is_initialized() {
                static bool initialized = false;

                return initialized;
            }

        public:
            Random() : m_lower(detail::zero<T>()), m_upper(detail::one<T>()) {
                using std::time;

                if (!is_initialized()) {
                    std::srand(static_cast<unsigned int>(std::time(0)));
                    is_initialized() = true;
                }
            }

            Random(T lower, T upper) : m_lower(lower), m_upper(upper) { self(); }

        private:
            Random(Random const &random) {}
            Random &operator=(Random const &random) { return *this; }

        public:
            T operator()() const {
                double rd = rand() / (1.0 * RAND_MAX);
                return static_cast<T>(m_lower + (m_upper - m_lower) * rd);
            }

            bool operator==(Random const &random) const {
                return (m_lower == random.m_lower && m_upper == random.m_upper);
            }
        };
    }
}

#endif