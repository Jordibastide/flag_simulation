#include <iostream>

#include <Utils/glm.hpp>
#include <Utils/WindowManager.hpp>

#include <Utils/renderer/FlagRenderer3D.hpp>
#include <Utils/renderer/Renderer3D.h>
#include <Utils/renderer/TrackballCamera.hpp>
#include <Utils/Sphere.hpp>
#include <Utils/Flag.h>



static const Uint32 WINDOW_WIDTH = 1024;
static const Uint32 WINDOW_HEIGHT = 768;

using namespace Utils;

class StaticParticleManager {
    std::vector<glm::vec3> m_PositionArray;
    std::vector<glm::vec3> m_ColorArray;

public:
    void addCircleParticles(float radius, uint32_t count) {
        float delta = 2 * 3.14f / count; // 2pi / nombre de particules
        for(size_t i = 0; i < count; ++i) {
            float c = cos(i * delta), s = sin(i * delta);
            addParticle(glm::vec3(radius * c, 0.f, radius * s), 1.f, glm::vec3(c, s, c * s));
        }
    }

    void addParticle(glm::vec3 position, float mass, glm::vec3 color) {
        m_PositionArray.push_back(position);
        m_ColorArray.push_back(color);
    }

    void drawParticles(Renderer3D& renderer) {
        renderer.drawParticles(m_PositionArray.size(),
                               m_PositionArray.data(),
                               m_ColorArray.data(),
                               0.05);
    }
};

int main() {
    WindowManager wm(WINDOW_WIDTH, WINDOW_HEIGHT, "Flag Simulation");
    wm.setFramerate(60);

    Flag flag(4096.f, 4, 3, 32, 16); // Flag creation
    glm::vec3 G(0.f, -0.001f, 0.f); // Gravity
    glm::vec3 W(0.02f, 0.f, 0.f); // Wind glm::sphericalRand(0.1f)

    FlagRenderer3D renderer(flag.gridWidth, flag.gridHeight);
    renderer.setProjMatrix(glm::perspective(70.f, float(WINDOW_WIDTH) / WINDOW_HEIGHT, 0.1f, 100.f));

    StaticParticleManager particleManager;
    particleManager.addParticle(glm::vec3(0), 1, glm::vec3(1, 1, 1));
    particleManager.addCircleParticles(0.05f, 128);

    Renderer3D sphererenderer;
    sphererenderer.setProjMatrix(glm::perspective(70.f, float(WINDOW_WIDTH) / WINDOW_HEIGHT, 0.1f, 100.f));

    TrackballCamera camera;
    int mouseLastX, mouseLastY;

    // Time between each frame
    float dt = 0.f;

    bool done = false;
    bool wireframe = false;
    while (!done) {
        wm.startMainLoop();

        // Render
        renderer.clear();
        sphererenderer.clear();

        //sphererenderer.setViewMatrix(camera.getViewMatrix());
        //particleManager.drawParticles(sphererenderer);

        renderer.setViewMatrix(camera.getViewMatrix());
        renderer.drawGrid(flag.positionArray.data(), wireframe);

        // Simulation
        if (dt > 0.f) {
            flag.applyExternalForce(G); // Gravity
            flag.applyExternalForce(W);
            //flag.applyExternalForce(glm::sphericalRand(0.04f)); // Random wind force
            flag.applyInternalForces(dt); // Internal forces
            flag.sphereCollision(glm::vec3(0,0,0), 1.f);
            //flag.autoCollisions();
            flag.update(dt); // Update system
        }

        // Events
        SDL_Event e;
        while (wm.pollEvent(e)) {
            switch (e.type) {
                default:
                    break;
                case SDL_QUIT:
                    done = true;
                    break;
                case SDL_KEYDOWN:
                    if (e.key.keysym.sym == SDLK_SPACE) {
                        wireframe = !wireframe;
                    }
                case SDL_MOUSEBUTTONDOWN:
                    if (e.button.button == SDL_BUTTON_WHEELUP) {
                        camera.moveFront(0.1f);
                    } else if (e.button.button == SDL_BUTTON_WHEELDOWN) {
                        camera.moveFront(-0.1f);
                    } else if (e.button.button == SDL_BUTTON_LEFT) {
                        mouseLastX = e.button.x;
                        mouseLastY = e.button.y;
                    }
            }
        }

        int mouseX, mouseY;
        if (SDL_GetMouseState(&mouseX, &mouseY) & SDL_BUTTON(SDL_BUTTON_LEFT)) {
            float dX = mouseX - mouseLastX, dY = mouseY - mouseLastY;
            camera.rotateLeft(glm::radians(dX));
            camera.rotateUp(glm::radians(dY));
            mouseLastX = mouseX;
            mouseLastY = mouseY;
        }

        // Window update
        dt = wm.update();
    }

    return EXIT_SUCCESS;
}
