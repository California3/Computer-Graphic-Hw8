#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Task 1.2): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.


//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        bool is_explicit = false; // Switch between explicit and implicit Euler

        for (auto &s : springs)
        {
            // TODO (Task 1.3): Use Hooke's law to calculate the force on a node
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Task 1.6): Add global damping
                // use f_d = -k_d * v, where f_d is the damping force, k_d is a damping constant, and v is the velocity of the mass

                if (is_explicit) {
                    // TODO (Task 1.3): Add the force due to gravity, then compute the new velocity and position (explicit Euler)

                } else {
                    // TODO (Task 1.3): Add the force due to gravity, then compute the new velocity and position (semi-implicit Euler)

                }
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Task 1.5): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 1.5): Set the new position of the rope mass
                
                // TODO (Part 1.6): Add global Verlet damping
            }
        }
    }
}
