#include <iostream>

#include <Utils/glm.hpp>
#include <Utils/WindowManager.hpp>
#include <Utils/renderer/FlagRenderer3D.hpp>
#include <Utils/renderer/TrackballCamera.hpp>
#include <Utils/Flag.h>

#include <AntTweakBar/AntTweakBar.h>
#include <AntTweakBar/atb.hpp>


static const Uint32 WINDOW_WIDTH = 1024;
static const Uint32 WINDOW_HEIGHT = 768;

using namespace Utils;


int main() {
    WindowManager wm(WINDOW_WIDTH, WINDOW_HEIGHT, "Flag Simulation");
    wm.setFramerate(60);

    TwInit(TW_OPENGL_CORE, NULL);
    TwWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);

    Flag flag(4096.f, 4, 3, 32, 16); // Flag creation
    glm::vec3 G(0.f, -0.002f, 0.f); // Gravity
    glm::vec3 W(0.02f, 0.f, -0.002f); // Wind

    FlagRenderer3D renderer(flag.gridWidth, flag.gridHeight);
    renderer.setProjMatrix(glm::perspective(70.f, float(WINDOW_WIDTH) / WINDOW_HEIGHT, 0.1f, 100.f));

    TrackballCamera camera;
    int mouseLastX, mouseLastY;

    // Init spheres
    std::vector<Sphere> spheres;
    spheres.push_back(Sphere(glm::vec3(-1.f,0,-0.1), 1.f));
    spheres.push_back(Sphere(glm::vec3(1.5,0,0.1), 0.5f));

    // Init GUI
    TwBar* gui = TwNewBar("Spheres and Wind parameters");

    TwAddVarRW(gui, "X", TW_TYPE_FLOAT, &spheres[0].center.x, " min=-5 max=5 step=0.1 group=Sphere1 label='X' ");
    TwAddVarRW(gui, "Y", TW_TYPE_FLOAT, &spheres[0].center.y, " min=-5 max=5 step=0.1 group=Sphere1 label='Y' ");
    TwAddVarRW(gui, "Z", TW_TYPE_FLOAT, &spheres[0].center.z, " min=-5 max=5 step=0.1 group=Sphere1 label='Z' ");
    TwAddVarRW(gui, "R", TW_TYPE_FLOAT, &spheres[0].radius, " min=0.1 max=5 step=0.1 group=Sphere1 label='Rayon' ");

    TwAddVarRW(gui, "X1", TW_TYPE_FLOAT, &spheres[1].center.x, " min=-5 max=5 step=0.1 group=Sphere2 label='X' ");
    TwAddVarRW(gui, "Y1", TW_TYPE_FLOAT, &spheres[1].center.y, " min=-5 max=5 step=0.1 group=Sphere2 label='Y' ");
    TwAddVarRW(gui, "Z1", TW_TYPE_FLOAT, &spheres[1].center.z, " min=-5 max=5 step=0.1 group=Sphere2 label='Z' ");
    TwAddVarRW(gui, "R1", TW_TYPE_FLOAT, &spheres[1].radius, " min=0.1 max=5 step=0.1 group=Sphere2 label='Rayon' ");

    TwAddVarRW(gui, "X2", TW_TYPE_FLOAT, &W.x, " min=-0.05 max=0.05 step=0.01 group=Wind label='X' ");
    TwAddVarRW(gui, "Y2", TW_TYPE_FLOAT, &W.y, " min=-0.05 max=0.05 step=0.01 group=Wind label='Y' ");
    TwAddVarRW(gui, "Z2", TW_TYPE_FLOAT, &W.z, " min=-0.05 max=0.05 step=0.01 group=Wind label='Z' ");


    // Time between each frame
    float dt = 0.f;

    bool done = false;
    bool wireframe = false;
    while (!done) {
        wm.startMainLoop();

        // Render
        renderer.clear();

        renderer.setViewMatrix(camera.getViewMatrix());
        renderer.drawGrid(flag.positionArray.data(), wireframe);

        // Simulation
        if (dt > 0.f) {
            flag.applyExternalForce(G); // Gravity
            flag.applyExternalForce(W); // Wind
            flag.applyInternalForces(dt); // Internal forces

            for(auto &sphere : spheres){
              flag.sphereCollision(sphere, dt);
            }

            flag.update(dt); // Update system
        }

        TwDraw();

        // Events
        SDL_Event e;
        while (wm.pollEvent(e)) {
            int handled = TwEventSDL(&e, SDL_MAJOR_VERSION, SDL_MINOR_VERSION);

            if(!handled) {
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

                int mouseX, mouseY;
                if (SDL_GetMouseState(&mouseX, &mouseY) & SDL_BUTTON(SDL_BUTTON_LEFT)) {
                    float dX = mouseX - mouseLastX, dY = mouseY - mouseLastY;
                    camera.rotateLeft(glm::radians(dX));
                    camera.rotateUp(glm::radians(dY));
                    mouseLastX = mouseX;
                    mouseLastY = mouseY;
                }
            }
        }

        // Window update
        dt = wm.update();
    }

    return EXIT_SUCCESS;
}
