// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_COMPUTE_ISOTROPIC_ELASTICITY_H
#define OPENTISSUE_FEM_COMPUTE_ISOTROPIC_ELASTICITY_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename real_type, typename vector_type>
            inline void compute_isotropic_elasticity(real_type const &young,
                real_type const &poisson, vector_type &D) {
                real_type poisson2 = 2.0 * poisson;
                real_type scale = young / ((1.0 + poisson) * (1.0 - poisson2));

                D(0) = (1.0 - poisson) * scale;
                D(1) = poisson * scale;
                D(2) = young / (2.0 + poisson2);
            }
        }
    }
}

#endif