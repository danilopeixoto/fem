// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_CLEAR_STIFFNESS_ASSEMBLY_H
#define OPENTISSUE_FEM_CLEAR_STIFFNESS_ASSEMBLY_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename node_iterator>
            inline void clear_stiffness_assembly(node_iterator begin, node_iterator end) {
                for (node_iterator node = begin; node != end; ++node) {
                    typedef typename node_iterator::value_type::matrix_iterator matrix_iterator;

                    node->m_f0.clear();

                    for (matrix_iterator Kij = node->Kbegin(); Kij != node->Kend(); ++Kij)
                        Kij->second.clear();
                }
            }
        }
    }
}

#endif