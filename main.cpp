
#define ARMA_DONT_USE_BLAS // it just works (tm)
#include <iostream>
#include <armadillo>
#include <glad/glad.h> // GUI
#include <GLFW/glfw3.h> // GUI
#include <imgui.h> // GUI
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h> // GUI-Plots
using namespace std;
using namespace arma;
#include "Log.h"
#include "Vehicle.h"
#include "VisualYmd.h"
#include "gui_backend.h"
#include "gui_color_themes.h"
#include "gui_frontend_panels.h"

int main() {
    Vehicle Car;
    //cout << Car.TireFy(vec({0, 0, 0, 0}),
    //                    vec({0.39, 1.28, -8.33, -8.10}),
    //                    vec({-652, -652, -745, -745}),
    //                    vec({0, 0, 0, 0})) << endl;
    //field<LogYmd> log = VisualYmdCV(Car, 10, 0, 0, 0, 1, 1);
    //cout << "log (0, 0) yaw: " << log(0, 0).yaw << endl;
    //cout << "log (0, 0) steer: " << log(0, 0).steer << endl;

    //cout << Car.LongLoadTransfer(vec({100, 100, 100, 100})) << endl;
    //double roll = 0;
    //double pitch = 0;
    //double heave = 0.0254;
    //cout << "Inclination:\n" << Car.GetInclination(0, roll, pitch, heave) << endl;
    //cout << "Wheel Steer:\n" << Car.GetSteer(-25, roll, pitch, heave) << endl;
    StartWindow();
    while (!glfwWindowShouldClose(graphicsWindow)){
        NewFrame();
            static UpdateYmd update_ymd;
            LeftPanel(Car, update_ymd);
            RightPanel(Car, update_ymd);
        RenderFrame();
    }
    EndWindow();

    return 0;
}