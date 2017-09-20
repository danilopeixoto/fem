// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_COMPUTE_B_H
#define OPENTISSUE_FEM_COMPUTE_B_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename real_type, typename vector_type>
            inline void compute_b(vector_type const &e10, vector_type const &e20,
                vector_type const &e30, real_type volume,
                vector_type *B) {
                real_type div6V = 1.0 / (6.0 * volume);

                B[1](0) = (e20(2) * e30(1) - e20(1) * e30(2)) * div6V;

                B[2](0) = (e10(1) * e30(2) - e10(2) * e30(1)) * div6V;
                B[3](0) = (e10(2) * e20(1) - e10(1) * e20(2)) * div6V;
                B[0](0) = -B[1](0) - B[2](0) - B[3](0);

                B[1](1) = (e20(0) * e30(2) - e20(2) * e30(0)) * div6V;
                B[2](1) = (e10(2) * e30(0) - e10(0) * e30(2)) * div6V;
                B[3](1) = (e10(0) * e20(2) - e10(2) * e20(0)) * div6V;
                B[0](1) = -B[1](1) - B[2](1) - B[3](1);

                B[1](2) = (e20(1) * e30(0) - e20(0) * e30(1)) * div6V;
                B[2](2) = (e10(0) * e30(1) - e10(1) * e30(0)) * div6V;
                B[3](2) = (e10(1) * e20(0) - e10(0) * e20(1)) * div6V;
                B[0](2) = -B[1](2) - B[2](2) - B[3](2);
            }
        }
    }
}

#endif