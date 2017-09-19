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
            inline void compute_isotropic_elasticity_vector(real_type const &young,
                real_type const &poisson, vector_type &D) {
                real_type poisson2 = 2.0 * poisson;
                real_type scale = young / ((1.0 + poisson) * (1.0 - poisson2));

                D(0) = (1.0 - poisson) * scale;
                D(1) = poisson * scale;
                D(2) = young / (2.0 + poisson2);
            }

            template <typename real_type, typename matrix_type>
            inline void compute_isotropic_elasticity_matrix(real_type const &young,
                real_type const &poisson,
                matrix_type &D) {
                assert(D.size1() == 6 || !"Incompatible matrix dimensions.");
                assert(D.size2() == 6 || !"Incompatible matrix dimensions.");

                typedef typename matrix_type::value_type value_type;

                value_type lambda = (poisson * young) / ((1 + poisson) * (1 - (2 * poisson)));
                value_type mu = young / (2 * (1 + poisson));
                value_type tmp = lambda + (2 * mu);

                D(0, 0) = tmp;
                D(0, 1) = lambda;
                D(0, 2) = lambda;
                D(0, 3) = 0;
                D(0, 4) = 0;
                D(0, 5) = 0;
                D(1, 0) = lambda;
                D(1, 1) = tmp;
                D(1, 2) = lambda;
                D(1, 3) = 0;
                D(1, 4) = 0;
                D(1, 5) = 0;
                D(2, 0) = lambda;
                D(2, 1) = lambda;
                D(2, 2) = tmp;
                D(2, 3) = 0;
                D(2, 4) = 0;
                D(2, 5) = 0;
                D(3, 0) = 0;
                D(3, 1) = 0;
                D(3, 2) = 0;
                D(3, 3) = mu;
                D(3, 4) = 0;
                D(3, 4) = 0;
                D(4, 0) = 0;
                D(4, 1) = 0;
                D(4, 2) = 0;
                D(4, 3) = 0;
                D(4, 4) = mu;
                D(4, 5) = 0;
                D(5, 0) = 0;
                D(5, 1) = 0;
                D(5, 2) = 0;
                D(5, 3) = 0;
                D(5, 4) = 0;
                D(5, 5) = mu;
            }

            template <typename real_type>
            inline void compute_isotropic_elasticity_vector(real_type const &young,
                real_type const &poisson, real_type &D00, real_type &D01, real_type &D33) {
                D01 = (poisson * young) / ((1 + poisson) * (1 - (2 * poisson)));
                D33 = young / (2 * (1 + poisson));
                D00 = D01 + (2 * D33);
            }
        }
    }
}

#endif