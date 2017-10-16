// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_INITIALIZE_H
#define OPENTISSUE_FEM_INITIALIZE_H

#include <opentissue/configuration.h>

#include <opentissue/fem/initialize_stiffness_plasticity.h>
#include <opentissue/fem/compute_mass.h>
#include <opentissue/fem/clear_stiffness_assembly.h>

namespace opentissue {
    namespace fem {
        template<typename fem_mesh, typename real_type>
        inline void initialize(fem_mesh &mesh, real_type const &density,
            real_type const &poisson, real_type const &young,
            real_type const &c_yield, real_type const &c_max_yield,
            real_type const &c_creep) {
            detail::initialize_stiffness_plasticity(mesh, density, poisson, young,
                c_yield, c_max_yield, c_creep);

            detail::compute_mass(mesh);
        }
    }
}

#endif