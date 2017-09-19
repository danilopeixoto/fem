// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_STIFFNESS_ASSEMBLY_H
#define OPENTISSUE_FEM_STIFFNESS_ASSEMBLY_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename tetrahedron_iterator>
            inline void stiffness_assembly(tetrahedron_iterator const &begin,
                tetrahedron_iterator const &end) {
                typedef typename tetrahedron_iterator::value_type::real_type real_type;
                typedef typename tetrahedron_iterator::value_type::vector_type vector_type;
                typedef typename tetrahedron_iterator::value_type::matrix_type matrix_type;
                typedef typename tetrahedron_iterator::value_type::node_iterator node_iterator;

                for (tetrahedron_iterator T = begin; T != end; ++T) {
                    matrix_type &Re = T->m_Re;

                    for (int i = 0; i < 4; ++i) {
                        node_iterator p_i = T->node(i);

                        vector_type f;
                        f.clear();

                        for (int j = 0; j < 4; ++j) {
                            node_iterator p_j = T->node(j);
                            matrix_type &Ke_ij = T->m_Ke[i][j];
                            vector_type &x0_j = p_j->m_model_coord;

                            f += Ke_ij * x0_j;

                            if (j >= i) {
                                matrix_type tmp = Re * Ke_ij * trans(Re);
                                p_i->K((int)p_j->idx()) += tmp;

                                if (j > i)
                                    p_j->K((int)p_i->idx()) += trans(tmp);
                            }
                        }

                        p_i->m_f0 -= Re * f;
                    }
                }
            }
        }
    }
}

#endif