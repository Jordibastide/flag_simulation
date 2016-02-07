#include <iostream>
#include "Utils/Flag.h"


inline glm::vec3 hookForce(float K, float L, const glm::vec3& P1, const glm::vec3& P2) {
    static const float epsilon = 0.0001;

    glm::vec3 F = K * (1 - L / glm::max(glm::distance(P1, P2), epsilon)) * (P2 - P1);
    return F;
}

inline glm::vec3 brakeForce(float V, float dt, const glm::vec3& v1, const glm::vec3& v2) {
    glm::vec3 F = V * ( v2 - v1) / dt;
    return F;
}

Flag::Flag(float mass, float width, float height, uint gridWidth, uint gridHeight):
        gridWidth(gridWidth), gridHeight(gridHeight),
        positionArray(gridWidth * gridHeight),
        velocityArray(gridWidth * gridHeight, glm::vec3(0.f)),
        massArray(gridWidth * gridHeight, mass / (gridWidth * gridHeight)),
        forceArray(gridWidth * gridHeight, glm::vec3(0.f))
{
    glm::vec3 origin(-0.5f * width, -0.5f * height, 0.f);
    glm::vec3 scale(width / (gridWidth - 1), height / (gridHeight - 1), 1.f);

    for(int j = 0; j < gridHeight; ++j)
    {
        for(int i = 0; i < gridWidth; ++i)
        {
            uint k = i + j * gridWidth;
            positionArray[k] = origin + glm::vec3(i, j, origin.z) * scale;
        }
    }

    // Compute distances
    L0.x = scale.x;
    L0.y = scale.y;
    std::cout << L0.x << ", " << L0.y << std::endl;
    L1 = glm::length(L0);
    L2 = 2.f * L0;

    // Fix this parameters
    K0 = 1.f;
    K1 = 1.f;
    K2 = 1.f;

    V0 = 0.3f;
    V1 = 0.1f;
    V2 = 0.1f;
}

void Flag::applyInternalForces(float dt)
{
    uint k;

    // Topology 0
    for(int j = 0; j < gridHeight - 1; ++j)
    {
        for(int i = 0; i < gridWidth - 1; ++i)
        {
            k = i + j * gridWidth;

            glm::vec3 horizontalHookForce = hookForce(K0, L0.x, positionArray[k], positionArray[k + 1]);
            glm::vec3 verticalHookForce = hookForce(K0, L0.y, positionArray[k], positionArray[k + gridWidth]);

            glm::vec3 horizontalBrakeForce = brakeForce(V0, dt, velocityArray[k], velocityArray[k + 1]);
            glm::vec3 verticalBrakeForce = brakeForce(V0, dt, velocityArray[k], velocityArray[k + gridWidth]);

            if (i != 0) {
                forceArray[k]     -= horizontalHookForce;
                forceArray[k]     -= verticalHookForce;
                forceArray[k]     += horizontalBrakeForce;
                forceArray[k]     += verticalBrakeForce;
                forceArray[k + gridWidth] += verticalHookForce;
                forceArray[k + gridWidth] -= verticalBrakeForce;

            }
            forceArray[k + 1] += horizontalHookForce;
            forceArray[k + 1] -= horizontalBrakeForce;

        }
    }

    // Topology 1
}

void Flag::applyExternalForce(const glm::vec3& F)
{
    uint k;
    for(int j = 0; j < gridHeight; ++j)
    {
        for(int i = 0; i < gridWidth; ++i)
        {
            k = i + j * gridWidth;
            if(i != 0)
                forceArray[k] += F;
        }
    }
}

void Flag::update(float dt)
{
    uint k;
    for(int j = 0; j < gridHeight; ++j)
    {
        for(int i = 0; i < gridWidth; ++i)
        {
            k = i + j * gridWidth;
            velocityArray[k] += dt * forceArray[k] / massArray[k];
            positionArray[k] += dt * velocityArray[k];
            forceArray[k] = glm::vec3(0.f);
        }
    }
}
