// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_TETRAHEDRON_TRAITS_H
#define OPENTISSUE_FEM_TETRAHEDRON_TRAITS_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename math_types> class TetrahedronTraits {
            public:
                typedef typename math_types::real_type real_type;
                typedef typename math_types::vector_type vector_type;
                typedef typename math_types::matrix_type matrix_type;

            public:
                real_type m_young;
                real_type m_poisson;
                real_type m_density;
                real_type m_yield;
                real_type m_max_yield;
                real_type m_creep;

                real_type m_volume;

                vector_type m_e10;
                vector_type m_e20;
                vector_type m_e30;

                vector_type m_B[4];
                vector_type m_E;

                matrix_type m_Ke[4][4];
                matrix_type m_Re;

                real_type m_plastic[6];
            };
        }
    }
}

#endif