#include <iostream>
#include "Utils/Flag.h"


inline glm::vec3 hookForce(float K, float L, const glm::vec3 &P1, const glm::vec3 &P2) {
    static const float epsilon = 0.0001;

    glm::vec3 F = K * (1 - L / glm::max(glm::distance(P1, P2), epsilon)) * (P2 - P1);
    return F;
}

inline glm::vec3 brakeForce(float V, float dt, const glm::vec3 &v1, const glm::vec3 &v2) {
    glm::vec3 F = V * (v2 - v1) / dt;
    return F;
}

Flag::Flag(float mass, float width, float height, uint gridWidth, uint gridHeight) :
        gridWidth(gridWidth), gridHeight(gridHeight),
        positionArray(gridWidth * gridHeight),
        velocityArray(gridWidth * gridHeight, glm::vec3(0.f)),
        massArray(gridWidth * gridHeight, mass / (gridWidth * gridHeight)),
        forceArray(gridWidth * gridHeight, glm::vec3(0.f)) {
    glm::vec3 origin(-0.5f * width, -0.5f * height, 0.f);
    glm::vec3 scale(width / (gridWidth - 1), height / (gridHeight - 1), 1.f);

    for (int j = 0; j < gridHeight; ++j) {
        for (int i = 0; i < gridWidth; ++i) {
            uint k = i + j * gridWidth;
            positionArray[k] = origin + glm::vec3(i, j, origin.z) * scale;
        }
    }

    // Compute distances
    L0.x = scale.x;
    L0.y = scale.y;
    L1 = glm::length(L0);
    L2 = 2.f * L0;

    // Fix this parameters
    K0 = 1.0;
    K1 = 1.3;
    K2 = 0.8;

    V0 = 0.08;
    V1 = 0.005;
    V2 = 0.06;
}

void Flag::applyInternalForces(float dt) {

    uint i,j,k;
    glm::vec3 horizontalForces, verticalForces;
    glm::vec3 rightCrossForces, leftCrossForces;

    // Topology 0 : Direct Link

        for(j = 0; j < gridHeight-1; ++j) {
            for(i = 0; i < gridWidth-1; ++i) {

                k = i + j * gridWidth;

                horizontalForces = hookForce(K0, L0.x, positionArray[k], positionArray[k + 1]) +
                                   brakeForce(V0, dt, velocityArray[k], velocityArray[k + 1]);

                verticalForces   = hookForce(K0, L0.y, positionArray[k], positionArray[k + gridWidth]) +
                                   brakeForce(V0, dt, velocityArray[k], velocityArray[k + gridWidth]);

                if(i>0){
                    forceArray[k]           += horizontalForces + verticalForces;
                    forceArray[k+gridWidth] -= verticalForces;
                }

                forceArray[k+1] -= horizontalForces;
            }
        }

        // Last Column
        for(j = 0; j < gridHeight-1; ++j) {

            k = j + (j+1) * (gridWidth-1);

            verticalForces = hookForce(K0, L0.y, positionArray[k], positionArray[k + gridWidth]) +
                             brakeForce(V0, dt, velocityArray[k], velocityArray[k + gridWidth]);

            forceArray[k]             += verticalForces;
            forceArray[k + gridWidth] -= verticalForces;
        }

        // Last line
        for(i = 0; i < gridWidth-1; ++i){

            k = i + (gridWidth) * (gridHeight-1);

            horizontalForces = hookForce(K0, L0.x, positionArray[k], positionArray[k + 1]) +
                               brakeForce(V0, dt, velocityArray[k], velocityArray[k + 1]);

            if(i>0)
                forceArray[k] += horizontalForces;

            forceArray[k+1] -= horizontalForces;
        }

    // Topology 1 : Cross link

        for(j = 0; j < gridHeight-1; ++j) {
            for(i = 0; i < gridWidth-1; ++i) {

                k = i + j * gridWidth;

                rightCrossForces = hookForce(K1, L1, positionArray[k], positionArray[k + gridWidth + 1]) +
                                   brakeForce(V1, dt, velocityArray[k], velocityArray[k + gridWidth + 1]);

                leftCrossForces  = hookForce(K1, L1, positionArray[k], positionArray[k + gridWidth - 1]) +
                                   brakeForce(V1, dt, velocityArray[k], velocityArray[k + gridWidth - 1]);

                if(i>0){
                    forceArray[k] += rightCrossForces + leftCrossForces;
                }

                if(i>1){
                    forceArray[k-1+gridWidth] -= leftCrossForces;
                }

                forceArray[k+1+gridWidth] -= rightCrossForces;
            }
        }

        // Last column
        for(j = 0; j < gridHeight-1; ++j) {

            k = j + (j+1) * (gridWidth-1);

            leftCrossForces = hookForce(K1, L1, positionArray[k], positionArray[k + gridWidth - 1]) +
                              brakeForce(V1, dt, velocityArray[k], velocityArray[k + gridWidth - 1]);

            forceArray[k]             += leftCrossForces;
            forceArray[k-1+gridWidth] -= leftCrossForces;
        }

    // Topology 2 : 2-step link

        for(j = 0; j < gridHeight-2; ++j) {
            for(i = 0; i < gridWidth-2; ++i) {

                k = i + j * gridWidth;

                horizontalForces = hookForce(K2, L2.x, positionArray[k], positionArray[k + 2]) +
                                   brakeForce(V2, dt, velocityArray[k], velocityArray[k + 2]);

                verticalForces   = hookForce(K2, L2.y, positionArray[k], positionArray[k + 2 * gridWidth]) +
                                   brakeForce(V2, dt, velocityArray[k], velocityArray[k + 2 * gridWidth]);

                if(i>0){
                    forceArray[k]             += horizontalForces + verticalForces;
                    forceArray[k+2*gridWidth] -= verticalForces;
                }

                forceArray[k+2] -= horizontalForces;
            }
        }

        // Last columns
        for(j = 0; j < gridHeight-2; ++j) {

            k = j + (j+1) * (gridWidth-1);

            verticalForces = hookForce(K2, L2.y, positionArray[k], positionArray[k + 2 * gridWidth]) +
                             brakeForce(V2, dt, velocityArray[k], velocityArray[k + 2 * gridWidth]);

            forceArray[k]             += verticalForces;
            forceArray[k+2*gridWidth] -= verticalForces;

            verticalForces = hookForce(K2, L2.y, positionArray[k - 1], positionArray[k + 2 * gridWidth - 1]) +
                             brakeForce(V2, dt, velocityArray[k - 1], velocityArray[k + 2 * gridWidth - 1]);

            forceArray[k-1]             += verticalForces;
            forceArray[k+2*gridWidth-1] -= verticalForces;
        }

        // Last lines
        for(i = 0; i < gridWidth-2; ++i){

            k = i + (gridWidth) * (gridHeight-1);

            horizontalForces = hookForce(K2, L2.x, positionArray[k], positionArray[k + 2]) +
                               brakeForce(V2, dt, velocityArray[k], velocityArray[k + 2]);

            if(i>0){
                forceArray[k]   += horizontalForces;
                forceArray[k+2] -= horizontalForces;
            }

            horizontalForces = hookForce(K2, L2.x, positionArray[k - gridWidth], positionArray[k + 2 - gridWidth]) +
                               brakeForce(V2, dt, velocityArray[k - gridWidth], velocityArray[k + 2 - gridWidth]);
            if(i>0){
                forceArray[k-gridWidth]   += horizontalForces;
                forceArray[k+2-gridWidth] -= horizontalForces;
            }
        }
}

void Flag::applyExternalForce(const glm::vec3 &F) {
    uint k;
    for (int j = 0; j < gridHeight; ++j) {
        for (int i = 0; i < gridWidth; ++i) {
            k = i + j * gridWidth;
            if (i != 0)
                forceArray[k] += F;
        }
    }
}

void Flag::update(float dt) {
    uint k;
    for (int j = 0; j < gridHeight; ++j) {
        for (int i = 0; i < gridWidth; ++i) {
            k = i + j * gridWidth;
            if (i != 0) {
                velocityArray[k] += dt * forceArray[k] / massArray[k];
                positionArray[k] += dt * velocityArray[k];
            }
            forceArray[k] = glm::vec3(0.f);
        }
    }
}
