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
        }
    }
}

#endif