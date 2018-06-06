// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_COMPUTE_KE_H
#define OPENTISSUE_FEM_COMPUTE_KE_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename tetrahedron_iterator>
            inline void compute_ke(tetrahedron_iterator const &tetrahedron) {
                typedef typename tetrahedron_iterator::value_type::real_type real_type;
                typedef typename tetrahedron_iterator::value_type::vector_type vector_type;
                typedef typename tetrahedron_iterator::value_type::matrix_type matrix_type;

                vector_type const &E = tetrahedron->m_E;
                vector_type const *B = tetrahedron->m_B;
                matrix_type(*Ke)[4] = tetrahedron->m_Ke;

                for (unsigned int i = 0; i < 4; i++) {
                    for (unsigned int j = i; j < 4; j++) {
                        real_type const &bi = B[i](0);
                        real_type const &ci = B[i](1);
                        real_type const &di = B[i](2);
                        real_type const &bj = B[j](0);
                        real_type const &cj = B[j](1);
                        real_type const &dj = B[j](2);

                        Ke[i][j](0, 0) = E(0) * bi * bj + E(2) * (ci * cj + di * dj);
                        Ke[i][j](0, 1) = E(1) * bi * cj + E(2) * ci * bj;
                        Ke[i][j](0, 2) = E(1) * bi * dj + E(2) * di * bj;
                        Ke[i][j](1, 1) = E(0) * ci * cj + E(2) * (bi * bj + di * dj);
                        Ke[i][j](1, 2) = E(1) * ci * dj + E(2) * di * cj;
                        Ke[i][j](2, 2) = E(0) * di * dj + E(2) * (ci * cj + bi * bj);

                        if (j == i) {
                            Ke[i][j](1, 0) = Ke[i][j](0, 1);
                            Ke[i][j](2, 0) = Ke[i][j](0, 2);
                            Ke[i][j](2, 1) = Ke[i][j](1, 2);

                            Ke[i][j] *= tetrahedron->m_volume0;
                        }
                        else {
                            Ke[i][j](1, 0) = E(1) * ci * bj + E(2) * bi * cj;
                            Ke[i][j](2, 0) = E(1) * di * bj + E(2) * bi * dj;
                            Ke[i][j](2, 1) = E(1) * di * cj + E(2) * ci * dj;

                            Ke[i][j] *= tetrahedron->m_volume0;

                            Ke[j][i](0, 0) = Ke[i][j](0, 0);
                            Ke[j][i](0, 1) = Ke[i][j](1, 0);
                            Ke[j][i](0, 2) = Ke[i][j](2, 0);
                            Ke[j][i](1, 0) = Ke[i][j](0, 1);
                            Ke[j][i](1, 1) = Ke[i][j](1, 1);
                            Ke[j][i](1, 2) = Ke[i][j](2, 1);
                            Ke[j][i](2, 0) = Ke[i][j](0, 2);
                            Ke[j][i](2, 1) = Ke[i][j](1, 2);
                            Ke[j][i](2, 2) = Ke[i][j](2, 2);
                        }
                    }
                }
            }
        }
    }
}

#endif