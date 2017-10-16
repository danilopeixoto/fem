// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_NODE_TRAITS_H
#define OPENTISSUE_FEM_NODE_TRAITS_H

#include <opentissue/configuration.h>

#include <unordered_map>

namespace opentissue {
    namespace fem {
        namespace detail {
            template<typename math_types> class NodeTraits {
            public:
                typedef typename math_types::real_type real_type;
                typedef typename math_types::vector_type vector_type;
                typedef typename math_types::matrix_type matrix_type;
                typedef typename math_types::index_type index_type;

                typedef typename std::unordered_map<index_type, matrix_type> matrix_container;
                typedef typename matrix_container::iterator matrix_iterator;

            public:
                bool m_fixed;

                real_type m_mass;

                vector_type m_f0;
                vector_type m_f_external;
                vector_type m_velocity;

                vector_type m_coord;
                vector_type m_world_coord;

                matrix_container m_K_row;

                matrix_container m_A_row;
                vector_type m_b;

                vector_type m_update;
                vector_type m_prev;
                vector_type m_residual;

                matrix_iterator Kbegin() { return m_K_row.begin(); }
                matrix_iterator Kend() { return m_K_row.end(); }
                matrix_iterator Abegin() { return m_A_row.begin(); }
                matrix_iterator Aend() { return m_A_row.end(); }

                matrix_type &K(unsigned int const &column_idx) { return m_K_row[column_idx]; }
                matrix_type &A(unsigned int const &column_idx) { return m_A_row[column_idx]; }

                NodeTraits() : m_fixed(false) {}
            };
        }
    }
}

#endif