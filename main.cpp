
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
#include "diff.h"
#include "Log.h"
#include "Vehicle.h"
#include "VisualYmd.h"
#include "gui_backend.h"
#include "gui_color_themes.h"
#include "gui_frontend_panels.h"

int main() {
    Vehicle Car;
    //Car.LoadCalculatedAttributes();
    //cout << Car.SteerLoadTransfer(0) << endl;
    //cout << Car.SteerLoadTransfer(10) << endl;
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