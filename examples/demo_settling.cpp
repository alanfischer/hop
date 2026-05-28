// demo_settling.cpp — Visual proof of settling and deactivation.
// A dense grid of 150 spheres falls into a box.
// Active spheres are colored; sleeping spheres turn gray.

#include <hop/hop.h>
#include <raylib.h>
#include <vector>
#include <iostream>

using namespace std;

int main() {
    using T = float;
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "Hop Physics — Settling & Deactivation Demo");
    SetTargetFPS(60);

    hop::simulator<T> sim;
    sim.set_gravity({ 0, 0, -10.0f });
    
    // Increased iterations for the dense pile
    sim.set_solver_iterations(32);

    // 1. Create the Box (Floor + 4 Walls)
    auto create_wall = [&](hop::vec3<T> mins, hop::vec3<T> maxs) {
        auto wall = std::make_shared<hop::solid<T>>();
        wall->set_infinite_mass();
        wall->set_coefficient_of_gravity(0.0f);
        wall->add_shape(std::make_shared<hop::shape<T>>(hop::aa_box<T>(mins, maxs)));
        sim.add_solid(wall);
    };

    create_wall({ -5, -5, -1 }, { 5, 5, 0 });    // Floor
    create_wall({ -5, -5, 0 }, { -4.5, 5, 10 }); // Wall X-
    create_wall({ 4.5, -5, 0 }, { 5, 5, 10 });  // Wall X+
    create_wall({ -5, -5, 0 }, { 5, -4.5, 10 }); // Wall Y-
    create_wall({ -5, 4.5, 0 }, { 5, 5, 10 });  // Wall Y+

    // 2. Create the Spheres (150 in a grid)
    std::vector<std::shared_ptr<hop::solid<T>>> spheres;
    int count = 0;
    for (int z = 0; z < 6; ++z) {
        for (int y = 0; y < 5; ++y) {
            for (int x = 0; x < 5; ++x) {
                if (count >= 150) break;
                auto s = std::make_shared<hop::solid<T>>();
                s->set_mass(1.0f);
                s->set_coefficient_of_restitution(0.5f);
                s->set_coefficient_of_restitution_override(true);
                s->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(0.4f)));
                s->set_position({ x * 1.0f - 2.0f, y * 1.0f - 2.0f, 2.0f + z * 1.2f });
                sim.add_solid(s);
                spheres.push_back(s);
                count++;
            }
        }
    }

    Camera3D camera = { 0 };
    camera.position = (Vector3){ 15.0f, 15.0f, 15.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 2.0f };
    camera.up = (Vector3){ 0.0f, 0.0f, 1.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    int frames = 0;
    while (!WindowShouldClose()) {
        if (!IsKeyDown(KEY_SPACE)) {
            sim.update(1.0f / 60.0f);
        }
        
        frames++;
        if (frames % 60 == 0) {
            std::cout << "Active: " << sim.count_active_solids() << " / " << spheres.size() << std::endl;
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        
        // Draw Box
        DrawCubeWires((Vector3){0, 0, 5}, 10, 10, 10, DARKGRAY);

        // Draw Spheres
        for (auto& s : spheres) {
            auto pos = s->get_position();
            Color color = s->active() ? MAROON : GRAY;
            DrawSphere((Vector3){(float)pos.x, (float)pos.y, (float)pos.z}, 0.4f, color);
            DrawSphereWires((Vector3){(float)pos.x, (float)pos.y, (float)pos.z}, 0.41f, 8, 8, DARKGRAY);
        }

        EndMode3D();

        DrawText("MAROON = Active, GRAY = Sleeping", 10, 10, 20, DARKGRAY);
        DrawText(TextFormat("Active: %d / %d", sim.count_active_solids(), (int)spheres.size()), 10, 40, 20, BLUE);
        DrawText("Space: Pause", 10, 70, 20, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
