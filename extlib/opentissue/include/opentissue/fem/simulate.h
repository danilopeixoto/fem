// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_SIMULATE_H
#define OPENTISSUE_FEM_SIMULATE_H

#include <opentissue/configuration.h>

#include <opentissue/fem/add_plasticity_force.h>
#include <opentissue/fem/clear_stiffness_assembly.h>
#include <opentissue/fem/conjugate_gradients.h>
#include <opentissue/fem/dynamics_assembly.h>
#include <opentissue/fem/update_position.h>
#include <opentissue/fem/stiffness_assembly.h>
#include <opentissue/fem/update_orientation.h>

namespace opentissue {
    namespace fem {
        template <typename fem_mesh, typename real_type>
        inline void simulate(fem_mesh &mesh, real_type const &mass_damping,
            real_type const &stiffness_damping, real_type const &time_step,
            unsigned int max_iterations) {
            assert(mass_damping >= 0 || !"Mass damping coefficient "
                "must be non-negative.");
            assert(stiffness_damping >= 0 || !"Stiffness damping coefficient "
                "must be non-negative.");
            assert(time_step > 0 || !"Time-step must be positive.");
            assert(max_iterations > 0 || !"Maximum iterations must be positive.");

            detail::clear_stiffness_assembly(mesh.node_begin(), mesh.node_end());

            detail::update_orientation(mesh.tetrahedron_begin(), mesh.tetrahedron_end());

            detail::stiffness_assembly(mesh.tetrahedron_begin(), mesh.tetrahedron_end());
            detail::add_plasticity_force(mesh.tetrahedron_begin(), mesh.tetrahedron_end(),
                time_step);

            detail::dynamics_assembly(mesh, mass_damping, stiffness_damping, time_step);

            detail::conjugate_gradients(mesh, max_iterations);
            detail::update_position(mesh, time_step);
        }
    }
}

#endif