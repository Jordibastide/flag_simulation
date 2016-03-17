#pragma once

#include <Utils/glm.hpp>
#include <vector>

struct Sphere {
  glm::vec3 center;
  float radius;

  Sphere(glm::vec3 center, float radius): center(center), radius(radius){};
};

struct Flag {
    unsigned int gridWidth, gridHeight; // Grid size

    // Points physics properties
    std::vector<glm::vec3> positionArray;
    std::vector<glm::vec3> velocityArray;
    std::vector<float> massArray;
    std::vector<glm::vec3> forceArray;

    // Initial distances
    glm::vec2 L0;
    float L1;
    glm::vec2 L2;

    // Resistance parameters
    float K0, K1, K2;
    // Brake parameters
    float V0, V1, V2;

    Flag(float mass, float width, float height, uint gridWidth, uint gridHeight);

    // Compute internal forces except on fixed points
    void applyInternalForces(float dt);

    // Compute external forces (gravity, wind) except on fixed points
    void applyExternalForce(const glm::vec3 &F);

    // Update speed and position with Leapfrog method
    void update(float dt);

    // Compute autocollision forces
    void autoCollisions();

    // Sphere Collision
    void sphereCollision(const Sphere &sphere);
};
