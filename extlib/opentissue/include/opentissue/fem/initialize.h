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
        template <typename fem_mesh, typename real_type>
        inline void initialize(fem_mesh &mesh, real_type const &density,
            real_type const &poisson, real_type const &young,
            real_type const &c_yield, real_type const &c_max_yield,
            real_type const &c_creep) {
            assert(density > 0 || !"Density must be positive.");
            assert(young >= 0 || !"Youngs modulus must be non-negative.");
            assert(c_yield >= 0 && c_yield <= 1.0 ||
                !"Yield strength must be in the range of 0 to 1.");
            assert(c_max_yield >= 0 && c_max_yield <= 1.0 ||
                !"Maximum yield strength must be in the range of 0 to 1.");
            assert(c_creep >= 0 && c_creep <= 1.0 ||
                !"Creep rate must be in the range of 0 to 1.");

            detail::initialize_stiffness_plasticity(
                mesh.tetrahedron_begin(), mesh.tetrahedron_end(),
                density, poisson, young, c_yield, c_max_yield, c_creep);

            detail::compute_mass(mesh);
        }
    }
}

#endif