// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_COMPUTE_VOLUME_H
#define OPENTISSUE_FEM_COMPUTE_VOLUME_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename vector_type>
            inline typename vector_type::value_type
                compute_volume(vector_type const &e10, vector_type const &e20,
                    vector_type const &e30) {
                typedef typename vector_type::value_type real_type;
                real_type sixV = e10 * (e20 % e30);

                return sixV / 6.0;
            }

            template <typename vector_type>
            inline typename vector_type::value_type
                compute_volume(vector_type const &p0, vector_type const &p1,
                    vector_type const &p2, vector_type const &p3) {
                typedef typename vector_type::value_type real_type;

                real_type d = p0(0);
                real_type d4 = p0(1);
                real_type d8 = p0(2);
                real_type d1 = p1(0) - d;
                real_type d5 = p1(1) - d4;
                real_type d9 = p1(2) - d8;
                real_type d2 = p2(0) - d;
                real_type d6 = p2(1) - d4;
                real_type d10 = p2(2) - d8;
                real_type d3 = p3(0) - d;
                real_type d7 = p3(1) - d4;
                real_type d11 = p3(2) - d8;
                real_type d12 = (d1 * d6 * d11 + d2 * d7 * d9 + d3 * d5 * d10) -
                    d1 * d7 * d10 - d2 * d5 * d11 - d3 * d6 * d9;

                return d12 / 6.0;
            }
        }
    }
}

#endif