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
        for (int i = 0; i < num_nodes; i++) {
            // Calculate the position of the current node
            Vector2D position = start + (end - start) * (float) i / (float) (num_nodes - 1);
            // push initial mass, set pinned to false by default.
            masses.push_back(new Mass(position, node_mass, false));
            // push spring, if not the first node.
            if (i > 0) {
                springs.push_back(new Spring(masses[i - 1], masses[i], k));
            }
        }

        // Comment-in this part when you implement the constructor
       for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
       }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        bool is_explicit = false; // Switch between explicit and implicit Euler

        for (auto &s : springs)
        {
            // TODO (Task 1.3): Use Hooke's law to calculate the force on a node
            double l_t = (s->m2->position - s->m1->position).norm();
            double l_delta = l_t - s->rest_length;
            Vector2D f_spring_ab = - s->k * ((s->m2->position - s->m1->position) / l_t) * l_delta;

            s->m1->forces += -f_spring_ab;
            s->m2->forces += f_spring_ab;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Task 1.6): Add global damping
                // use f_d = -k_d * v, where f_d is the damping force, k_d is a damping constant, and v is the velocity of the mass
                float kd = 0.00005;
                m->forces += - kd * m->velocity;

                if (is_explicit) {
                    // TODO (Task 1.3): Add the force due to gravity, then compute the new velocity and position (explicit Euler)
                    m->forces += gravity * m->mass;

                    m->position += m->velocity * delta_t;
                    m->velocity += (m->forces / m->mass) * delta_t;

                } else {
                    // TODO (Task 1.3): Add the force due to gravity, then compute the new velocity and position (semi-implicit Euler)
                    m->forces += gravity * m->mass;

                    m->velocity += (m->forces / m->mass) * delta_t;
                    m->position += m->velocity * delta_t;
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
            Vector2D dir_v1_to_v2 = s->m2->position - s->m1->position;
            double l_t = (dir_v1_to_v2).norm();
            double l_delta = l_t - s->rest_length;

            if(!s->m1->pinned) {
                s->m1->position += (l_delta / l_t) * dir_v1_to_v2 * 0.5;
            }

            if(!s->m2->pinned) {
                s->m2->position += (l_delta / l_t) * -dir_v1_to_v2 * 0.5;
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 1.5): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                // TODO (Part 1.6): Add global Verlet damping
                float kd = 0.00005;

                m->position = m->position + (1-kd) * (m->position - m->last_position) + (m->forces / m->mass) * delta_t * delta_t;
                m->last_position = temp_position;
            }
             m->forces = Vector2D(0, 0);
        }
    }
}
