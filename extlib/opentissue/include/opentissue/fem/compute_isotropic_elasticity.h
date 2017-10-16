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
            template<typename tetrahedron_iterator>
            inline void compute_isotropic_elasticity(tetrahedron_iterator const &tetrahedron) {
                typedef typename tetrahedron_iterator::value_type::real_type real_type;
                typedef typename tetrahedron_iterator::value_type::vector_type vector_type;

                real_type const &poisson = tetrahedron->m_poisson;
                real_type const &young = tetrahedron->m_young;

                vector_type & E = tetrahedron->m_E;

                real_type poisson2 = 2.0 * poisson;
                real_type scale = young / ((1.0 + poisson) * (1.0 - poisson2));

                E(0) = (1.0 - poisson) * scale;
                E(1) = poisson * scale;
                E(2) = young / (2.0 + poisson2);
            }
        }
    }
}

#endif