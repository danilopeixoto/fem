// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MATH_BASIC_TYPES_H
#define OPENTISSUE_MATH_BASIC_TYPES_H

#include <opentissue/configuration.h>

#include <opentissue/math/matrix.h>
#include <opentissue/math/value_traits.h>
#include <opentissue/math/vector.h>

namespace opentissue {
    namespace math {
        template <typename real_type_, typename index_type_> class BasicMathTypes {
        public:
            typedef real_type_ real_type;
            typedef index_type_ index_type;
            typedef Vector<real_type> vector_type;
            typedef Matrix<real_type> matrix_type;
            typedef Vector<index_type> index_vector_type;
            typedef ValueTraits<real_type> value_traits;
        };

        typedef BasicMathTypes<double, unsigned int> default_math_types;
    }
}

#endif