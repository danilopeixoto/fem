// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MESH_DEFAULT_TRAITS_H
#define OPENTISSUE_MESH_DEFAULT_TRAITS_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace mesh {
        template<typename math_types> class DefaultNodeTraits {
        public:
            typedef typename math_types::vector_type vector_type;
            typedef typename math_types::real_type real_type;

            vector_type m_coord;
        };

        class DefaultTetrahedronTraits {};
    }
}

#endif