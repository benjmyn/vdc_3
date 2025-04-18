//
// Created by benjmyn on 3/29/25.
//

#ifndef GUI_FRONTEND_PANELS_H
#define GUI_FRONTEND_PANELS_H
#include "diff.h"
#include "Log.h"
#include "VisualYmd.h"
static void HelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip())
    {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}
struct UpdateYmd {
    bool is_true = false;
    int type_ymd = 0; // 0: const V, 1: const R, 2: LTS
    int type_tire = 1; // 0: Fx, 1: Fy, 2: Mz
    // Needs floats for sliders
    float v = 20;
    float R = 100;
    float T = 0;
    float yaw_range = 6;
    float steer_range = 60;
    int yaw_ct = 55;
    int steer_ct = 55;
    int refines = 10;
};
// YMD plots
inline void YmdTooltip(const LogYmd &log) {
    ImGui::BeginTooltip();
    ImGui::SeparatorText("Main Dynamics");
    {
        ImGui::BeginGroup();
        ImGui::Text("v = %.1f m/s ", log.v);
        ImGui::Text("R = %.2f m ", log.R);
        ImGui::Text("ax = %.3f G ", log.ax/9.81);
        ImGui::Text("ay = %.3f G ", log.ay/9.81);
        ImGui::Text("aa = %.2f rad.s-2 ", log.aa);
        ImGui::EndGroup();
    }
    ImGui::SameLine();
    {
        ImGui::BeginGroup();
        ImGui::Text("yaw = %+.2f° ", log.yaw);
        ImGui::Text("steer = %+.2f° ", log.steer);
        ImGui::Text("roll = %+.2f° ", log.roll);
        ImGui::Text("pitch = %+.2f° ", log.pitch);
        ImGui::Text("heave = %+.0f mm ", 1e3 * log.heave);
        ImGui::EndGroup();
    }
    ImGui::SeparatorText("Alignment");
    {
        ImGui::BeginGroup();
        ImGui::Text("cam = %+.2f° %+.2f° \n", log.cam(0), log.cam(1));
        ImGui::Text("      %+.2f° %+.2f° \n", log.cam(2), log.cam(3));
        ImGui::Text("slip = %+.2f° %+.2f° \n", log.slip(0), log.slip(1));
        ImGui::Text("       %+.2f° %+.2f° \n", log.slip(2), log.slip(3));
        ImGui::EndGroup();
    }
    ImGui::SameLine();
    {
        ImGui::BeginGroup();
        ImGui::Text("toe = %+.2f° %+.2f° \n", log.toe(0), log.toe(1));
        ImGui::Text("      %+.2f° %+.2f° \n", log.toe(2), log.toe(3));
        ImGui::Text("disp = %+.0f mm %+.0f mm \n", 1e3 * log.bump(0), 1e3 * log.bump(1));
        ImGui::Text("       %+.0f mm %+.0f mm \n", 1e3 * log.bump(2), 1e3 * log.bump(3));
        ImGui::EndGroup();
    }
    ImGui::SeparatorText("Forces");
    {
        ImGui::BeginGroup();
        ImGui::Text("Tw = %+3.0f N.m %+3.0f N.m \n", log.Tw(0), log.Tw(1));
        ImGui::Text("     %+3.0f N.m %+3.0f N.m \n", log.Tw(2), log.Tw(3));
        ImGui::Text("fz = %.0f N %.0f N \n", -log.fz(0), -log.fz(1));
        ImGui::Text("     %.0f N %.0f N \n", -log.fz(2), -log.fz(3));
        ImGui::EndGroup();
    }
    ImGui::SameLine();
    {
        ImGui::BeginGroup();
        ImGui::Text("fy = %+.0f N %+.0f N \n", log.fyt(0), log.fyt(1));
        ImGui::Text("     %+.0f N %+.0f N \n", log.fyt(2), log.fyt(3));
        ImGui::Text("mz = %+.0f N.m %+.0f N.m \n", log.mz(0), log.mz(1));
        ImGui::Text("     %+.0f N.m %+.0f N.m \n", log.mz(2), log.mz(3));
        ImGui::EndGroup();
    }
    ImGui::EndTooltip();
}
inline void PlotYmdTooltip(const field<LogYmd> &log) {
    bool tipped_off = false;
    for (int i = 0; i < log.n_rows; ++i) {
        for (int j = 0; j < log.n_cols; ++j) {
            ImVec2 point_pos = ImPlot::PlotToPixels(ImPlotPoint(log(i, j).ay, log(i, j).aa));
            ImVec2 mouse_pos = ImPlot::PlotToPixels(ImPlot::GetPlotMousePos());
            double dx = mouse_pos.x - point_pos.x;
            double dy = mouse_pos.y - point_pos.y;
            if (!tipped_off && sqrt(dx*dx+dy*dy) < 5) {
                YmdTooltip(log(i,j));
                tipped_off = true;
            }
        }
    }
}
inline void PlotYmdLines(const field<LogYmd> &log, const ImVec4 &col_yaw_start, const ImVec4 &col_yaw_end, const ImVec4 &col_steer_start, const ImVec4 &col_steer_end) {
    ImVec4 color;
    float rat;
    for (int i = 0; i < log.n_rows; ++i) { // Constant yaw, vary steer
        string name = "##yaw_"+to_string(i);
        double ay_arr[log.n_cols];
        double aa_arr[log.n_cols];
        for (int j = 0; j < log.n_cols; ++j) {
            ay_arr[j] = log(i, j).ay;
            aa_arr[j] = log(i, j).aa;
        }
        rat = (float)i / (float)(log.n_rows - 1);
        color = ImVec4(col_yaw_start.x * (1-rat) + col_yaw_end.x * rat,
                      col_yaw_start.y * (1-rat) + col_yaw_end.y * rat,
                      col_yaw_start.z * (1-rat) + col_yaw_end.z * rat, 255);
        ImPlot::PushStyleColor(ImPlotCol_Line, ImGui::ColorConvertFloat4ToU32(color));
        ImPlot::PlotLine(name.c_str(), ay_arr, aa_arr, log.n_cols);
        ImPlot::PopStyleColor();
    }
    for (int j = 0; j < log.n_cols; ++j) { // Very yaw, constant steer
        string name = "##steer_"+to_string(j);
        double ay_arr[log.n_rows];
        double aa_arr[log.n_rows];
        for (int i = 0; i < log.n_rows; ++i) {
            ay_arr[i] = log(i, j).ay;
            aa_arr[i] = log(i, j).aa;
        }
        rat = (float)j / (float)(log.n_cols - 1);
        color = ImVec4(col_steer_start.x * (1-rat) + col_steer_end.x * rat,
                      col_steer_start.y * (1-rat) + col_steer_end.y * rat,
                      col_steer_start.z * (1-rat) + col_steer_end.z * rat, 255);
        ImPlot::PushStyleColor(ImPlotCol_Line, ImGui::ColorConvertFloat4ToU32(color));
        ImPlot::PlotLine(name.c_str(), ay_arr, aa_arr, log.n_rows);
        ImPlot::PopStyleColor();
    }
}
// Stability plots
inline void PlotStabLines(const field<LogYmd> &log, const ImVec4 &col_steer_start, const ImVec4 &col_steer_end) {
    ImVec4 color; // Constant steer
    float rat;
    for (int j = 0; j < log.n_cols; ++j) { // For each column (steer) plot a line (stability vs. yaw)
        string name = "##stability_"+to_string(j);
        double yaw_arr[log.n_rows];
        double aa_arr[log.n_rows];
        double dadb_arr[log.n_rows];
        for (int i = 0; i < log.n_rows; ++i) {
            yaw_arr[i] = log(i, j).yaw;
            aa_arr[i] = log(i, j).aa;
        }
        diffdb(dadb_arr, yaw_arr, aa_arr, log.n_rows);
        rat = (float)j / (float)(log.n_cols - 1);
        color = ImVec4(col_steer_start.x * (1-rat) + col_steer_end.x * rat,
                      col_steer_start.y * (1-rat) + col_steer_end.y * rat,
                      col_steer_start.z * (1-rat) + col_steer_end.z * rat, 255);
        ImPlot::PushStyleColor(ImPlotCol_Line, ImGui::ColorConvertFloat4ToU32(color));
        ImPlot::PlotLine(name.c_str(), yaw_arr, dadb_arr, log.n_rows);
        ImPlot::PopStyleColor();
    }
}
inline void PlotYawLines(const field<LogYmd> &log, const ImVec4 &col_steer_start, const ImVec4 &col_steer_end) {
    ImVec4 color; // Constant steer
    float rat;
    for (int j = 0; j < log.n_cols; ++j) { // For each column (steer) plot a line (stability vs. yaw)
        string name = "##yaw_"+to_string(j);
        double yaw_arr[log.n_rows];
        double aa_arr[log.n_rows];
        //double dadb_arr[log.n_rows];
        for (int i = 0; i < log.n_rows; ++i) {
            yaw_arr[i] = log(i, j).yaw;
            aa_arr[i] = log(i, j).aa;
        }
        //diffdb(dadb_arr, yaw_arr, aa_arr, log.n_rows);
        rat = (float)j / (float)(log.n_cols - 1);
        color = ImVec4(col_steer_start.x * (1-rat) + col_steer_end.x * rat,
                      col_steer_start.y * (1-rat) + col_steer_end.y * rat,
                      col_steer_start.z * (1-rat) + col_steer_end.z * rat, 255);
        ImPlot::PushStyleColor(ImPlotCol_Line, ImGui::ColorConvertFloat4ToU32(color));
        ImPlot::PlotLine(name.c_str(), yaw_arr, aa_arr, log.n_rows);
        ImPlot::PopStyleColor();
    }
}
inline void PlotContLines(const field<LogYmd> &log, const ImVec4 &col_yaw_start, const ImVec4 &col_yaw_end) {
    ImVec4 color;
    float rat;
    for (int i = 0; i < log.n_rows; ++i) { // Constant yaw, vary steer
        string name = "##control_"+to_string(i);
        double steer_arr[log.n_cols];
        double aa_arr[log.n_cols];
        double dads_arr[log.n_cols];
        for (int j = 0; j < log.n_cols; ++j) {
            steer_arr[j] = log(i, j).steer;
            aa_arr[j] = log(i, j).aa;
        }
        diffdb(dads_arr, steer_arr, aa_arr, log.n_cols);
        rat = (float)i / (float)(log.n_rows - 1);
        color = ImVec4(col_yaw_start.x * (1-rat) + col_yaw_end.x * rat,
                      col_yaw_start.y * (1-rat) + col_yaw_end.y * rat,
                      col_yaw_start.z * (1-rat) + col_yaw_end.z * rat, 255);
        ImPlot::PushStyleColor(ImPlotCol_Line, ImGui::ColorConvertFloat4ToU32(color));
        ImPlot::PlotLine(name.c_str(), steer_arr, dads_arr, log.n_cols);
        ImPlot::PopStyleColor();
    }
}
inline void PlotSteerLines(const field<LogYmd> &log, const ImVec4 &col_yaw_start, const ImVec4 &col_yaw_end) {
    ImVec4 color;
    float rat;
    for (int i = 0; i < log.n_rows; ++i) { // Constant yaw, vary steer
        string name = "##steer_"+to_string(i);
        double steer_arr[log.n_cols];
        double aa_arr[log.n_cols];
        //double dads_arr[log.n_cols];
        for (int j = 0; j < log.n_cols; ++j) {
            steer_arr[j] = log(i, j).steer;
            aa_arr[j] = log(i, j).aa;
        }
        //diffdb(dads_arr, steer_arr, aa_arr, log.n_cols);
        rat = (float)i / (float)(log.n_rows - 1);
        color = ImVec4(col_yaw_start.x * (1-rat) + col_yaw_end.x * rat,
                      col_yaw_start.y * (1-rat) + col_yaw_end.y * rat,
                      col_yaw_start.z * (1-rat) + col_yaw_end.z * rat, 255);
        ImPlot::PushStyleColor(ImPlotCol_Line, ImGui::ColorConvertFloat4ToU32(color));
        ImPlot::PlotLine(name.c_str(), steer_arr, aa_arr, log.n_cols);
        ImPlot::PopStyleColor();
    }
}
// Tire plots
inline void SubplotsTireFx(Vehicle &car) {

}
inline void SubplotsTireFy(Vehicle &car) { // Passes tire data through the Vehicle object
    // Setup variables
    vec camber = {0, 2, 4};
    vec load = {-222, -445, -667, -889, -1112};
    //Col<string> loadLabel = {"50lb","100lb","150lb","200lb","250lb"};
    //Col<string> camberLabel = {"0°","-2°","-4°"};
    static Vehicle car_theo;
    car_theo.p94y = car.p94y;
    car_theo.p94ys = vec({1, 1});
    // Plots
    ImVec2 subplots_area = ImVec2(ImGui::GetContentRegionAvail().x * 5/3, ImGui::GetContentRegionAvail().y * 3/2);
    ImGui::BeginChild("##TireSubplotsArea", ImGui::GetContentRegionAvail(), false, ImGuiWindowFlags_HorizontalScrollbar);
    ImPlot::BeginSubplots("##TireSubplots", 3, 5, subplots_area);
    {
        for (int i = 0; i < 5; ++i) { // Load
            for (int j = 0; j < 3; ++j) { // Camber
                ImPlot::BeginPlot("##subplotij");
                ImPlot::EndPlot();
            }
        }
    }
    ImPlot::EndSubplots();
    ImGui::EndChild();
}
inline void SubplotsTireMz(Vehicle &car) {

}
// Left panel content
inline void InterfaceYmd(Vehicle &car, UpdateYmd &update_ymd) {
    // Title card itself
    const static string titlecard =
                    "     ___________   ____  ___  _____  _____  \n"
                    "    / __/ __/ _ | / __/__| | / / _ \\/ ___/ \n"
                    "   / _/_\\ \\/ __ |/ __/___| |/ / // / /__  \n"
                    "  /_/ /___/_/ |_|___/    |___/____/\\___/   \n"
                    "\n"
                    "  Formula SAE Vehicle Dynamics Collection  \n"
                    "       by Ben Mynhier | VTMotorsports      \n";
    ImGui::Text("\n\n%s\n", titlecard.c_str());
    // Change color theme
    static bool lm = false;
    if (lm){
        ImGui::SetCursorPos(ImVec2(177, ImGui::GetCursorPosY()));
        if (ImGui::Button("Dark Mode")) {
            lm = false;
        }
        ImGui::StyleColorsCustomLight();
        ImPlot::StyleColorsLight();
    }
    else if (!lm){
        ImGui::SetCursorPos(ImVec2(170, ImGui::GetCursorPosY()));
        if (ImGui::Button("Light Mode")) {
            lm = true;
        }
        ImGui::StyleColorsCustomDark();
        ImPlot::StyleColorsDark();
    }
    ImGui::NewLine();
    ImGui::Separator();
    // Toggleable modes
    ImGui::SetCursorPos(ImVec2(32 + ImGui::GetCursorPosX(), 12 + ImGui::GetCursorPosY()));
    ImGui::BeginGroup();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
    ImGui::RadioButton("Const. Velocity", &update_ymd.type_ymd, 0);
    ImGui::RadioButton("Const. Radius", &update_ymd.type_ymd, 1);
    ImGui::RadioButton("Lap Time Sim", &update_ymd.type_ymd, 2);
    ImGui::PopStyleVar();
    ImGui::EndGroup();
    ImGui::SameLine();
    // Generate button
    ImGui::BeginGroup();
    ImGui::SetCursorPos(ImVec2(48 + ImGui::GetCursorPosX(), 0 + ImGui::GetCursorPosY()));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12, 12));
    if (ImGui::Button("Generate\n  Data")){update_ymd.is_true = true;}
    ImGui::PopStyleVar();
    ImGui::EndGroup();

    // Put specifics here
    ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX(), 8 + ImGui::GetCursorPosY()));
    ImGui::SeparatorText("Plot Controls");
    ImGui::PushItemWidth(212);
    ImGui::BeginGroup(); // Left Group
    {
        switch (update_ymd.type_ymd) {
            case 0:
                ImGui::Text("Velocity:");
                ImGui::SliderFloat("##slider1l", &update_ymd.v, 10, 30, "%.0f m/s");
                break;
            case 1:
                ImGui::Text("Radius:");
                ImGui::SliderFloat("##slider1l", &update_ymd.R, 10, 100, "%.0f m");
                break;
            case 2:
                ImGui::Text("Track:");
                HelpMarker("Whoops! Not working yet!");
                break;
        }
        ImGui::Text("Yaw Range:");
        ImGui::SliderFloat("##slider2l", &update_ymd.yaw_range, 0, 15, "%.0f°");
        ImGui::Text("Steer Range:");
        ImGui::SliderFloat("##slider3l", &update_ymd.steer_range, 0, 130, "%.0f°");
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX(), -3 + ImGui::GetCursorPosY()));
    ImGui::BeginGroup(); // Right Group
    {
        ImGui::Text("Applied Torque:");
        ImGui::SliderFloat("##slider1r", &update_ymd.T, -1000, 800, "%.0f N.m");
        ImGui::Text("Yaw Isolines:");
        ImGui::SliderInt("##slider2r", &update_ymd.yaw_ct, 10, 100, "%i");
        ImGui::Text("Steer Isolines:");
        ImGui::SliderInt("##slider3r", &update_ymd.steer_ct, 10, 100, "%i");
    }
    ImGui::EndGroup();
    ImGui::PopItemWidth();
    ImGui::Separator();
}
inline void InterfaceCar(Vehicle &car, UpdateYmd &update_ymd) {
    ImGui::BeginChild("Scroll Zone", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y - 50), false);
    ImGui::PushItemWidth(150);
    ImGui::SeparatorText("Mass Properties");
    ImGui::InputDouble("Mass [kg]", &car.m, 1.0f, 10.0f, "%.1f");
    ImGui::InputDouble("Inertia [kg.m2]", &car.izz, 1.0f, 10.0f, "%.1f");
    ImGui::InputDouble("Center of Gravity Height [m]", &car.h, 0.001f, 0.01f, "%.4f");
    ImGui::InputDouble("Front Weight [-]", &car.fwt, 0.001f, 0.01f, "%.3f");
    //
    ImGui::SeparatorText("Footprint");
    ImGui::InputDouble("Wheelbase [m]", &car.l, 0.001f, 0.01f, "%.3f");
    ImGui::InputDouble("Front Track [m]", &car.tf, 0.001f, 0.01f, "%.3f");
    ImGui::InputDouble("Rear Track [m]", &car.tr, 0.001f, 0.01f, "%.3f");
    //
    ImGui::SeparatorText("Geometry");
    ImGui::InputDouble("Front Roll Center Height [m]", &car.zf, 0.001f, 0.01f, "%.3f");
    ImGui::InputDouble("Rear Roll Center Height [m]", &car.zr, 0.001f, 0.01f, "%.3f");
    ImGui::InputDouble("Front Camber Gain [°/m]", &car.cam_gain_f, 1.0f, 10.0f, "%.1f");
    ImGui::InputDouble("Rear Camber Gain [°/m]", &car.cam_gain_r, 1.0f, 10.0f, "%.1f");
    ImGui::InputDouble("Front Toe Gain [°/m]", &car.toe_gain_f, 1.0f, 10.0f, "%.1f");
    ImGui::InputDouble("Rear Toe Gain [°/m]", &car.toe_gain_r, 1.0f, 10.0f, "%.1f");
    ImGui::InputDouble("Ackermann Rate [°/°]", &car.ack, 0.01f, 0.1f, "%.2f");
    ImGui::InputDouble("Front Motion Ratio [-]", &car.mrsf, 0.001f, 0.01f, "%.3f");
    ImGui::InputDouble("Rear Motion Ratio [-]", &car.mrsr, 0.001f, 0.01f, "%.3f");
    ImGui::InputDouble("Front ARB Motion Ratio [-]", &car.mraf, 0.001f, 0.01f, "%.3f");
    ImGui::InputDouble("Rear ARB Motion Ratio [-]", &car.mrar, 0.001f, 0.01f, "%.3f");
    //
    ImGui::SeparatorText("Alignment");
    ImGui::InputDouble("Front Camber [°]", &car.camf, 0.1f, 1.0f, "%.1f");
    ImGui::InputDouble("Rear Camber [°]", &car.camr, 0.1f, 1.0f, "%.1f");
    ImGui::InputDouble("Front Toe [°]", &car.toef, 0.01f, 0.1f, "%.2f");
    ImGui::InputDouble("Rear Toe [°]", &car.toer, 0.01f, 0.1f, "%.2f");
    //
    ImGui::SeparatorText("Springing");
    ImGui::InputDouble("Front Spring [N/m]", &car.ksf, 100.0f, 1000.0f, "%.0f");
    ImGui::InputDouble("Rear Spring [N/m]", &car.ksr, 100.0f, 1000.0f, "%.0f");
    ImGui::InputDouble("Front ARB Stiffness [N/m]", &car.kaf, 100.0f, 1000.0f, "%.0f");
    ImGui::InputDouble("Rear ARB Stiffness [N/m]", &car.kar, 100.0f, 1000.0f, "%.0f");
    //
    ImGui::PopItemWidth();
    ImGui::EndChild();
    // JUST the generate button jesus this is a lot of code for one thing
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12, 12));
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0.65, 0, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0, 0.80, 0, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0, 0.7, 0, 1));
    ImGui::SetCursorPos(ImVec2(140,ImGui::GetCursorPosY() + 5));
    if (ImGui::Button("Generate Data")){update_ymd.is_true = true;}
    ImGui::PopStyleColor(3);
    ImGui::PopStyleVar();
}
inline void InterfaceTire(Vehicle &car, UpdateYmd &update_ymd) {

}
inline void InterfaceTv(Vehicle &car, UpdateYmd &update_ymd) {
    ImGui::BeginChild("Scroll Zone", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y - 50), false);
    ImGui::PushItemWidth(150);
    ImGui::SeparatorText("Motor Limits");
    ImGui::Checkbox("Enable Torque Vectoring", &car.tv_enabled);
    ImGui::SeparatorText("Motor Control Parameters");
    ImGui::InputDouble("Steer Effect (Front) [Nm/deg]", &car.dTf_str, 0.1f, 1.0f, "%.1f");
    ImGui::InputDouble("Steer Effect (Rear) [Nm/deg]", &car.dTr_str, 0.1f, 1.0f, "%.1f");
    ImGui::PopItemWidth();
    ImGui::EndChild();
}
// Right panel content
inline void TabYmd(Vehicle &car, UpdateYmd &update_ymd) {
    ImPlot::BeginPlot("Yaw Moment Diagram", ImGui::GetContentRegionAvail());
    // Setup limits to initialize at consistent given values, but
    ImPlot::SetupAxis(ImAxis_X1, "Lateral Accel (m.s-2)", ImPlotAxisFlags_None);
    ImPlot::SetupAxisLimits(ImAxis_X1, -20.0, 20.0, ImGuiCond_FirstUseEver);
    ImPlot::SetupAxis(ImAxis_Y1, "Yaw Accel (rad.s-2)", ImPlotAxisFlags_None);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -25.0, 25.0, ImGuiCond_FirstUseEver);
    // YMD
    static field<LogYmd> log = VisualYmdCV(car, update_ymd.refines, update_ymd.v, update_ymd.T, update_ymd.yaw_range, update_ymd.steer_range, update_ymd.yaw_ct, update_ymd.steer_ct);
    if (update_ymd.is_true) {
        if (update_ymd.type_ymd == 0) {
            log = VisualYmdCV(car, update_ymd.refines, update_ymd.v, update_ymd.T, update_ymd.yaw_range, update_ymd.steer_range, update_ymd.yaw_ct, update_ymd.steer_ct);
        }
        else if (update_ymd.type_ymd == 1) {
            log = VisualYmdCR(car, update_ymd.refines, update_ymd.R, update_ymd.T, update_ymd.yaw_range, update_ymd.steer_range, update_ymd.yaw_ct, update_ymd.steer_ct);
        }
        update_ymd.is_true = false;
    }
    // YMD
    PlotYmdLines(log, ImVec4(1, 0, 0, 1), ImVec4(0.5, 1, 0, 1),
                        ImVec4(0, 0, 1, 1), ImVec4(0, 1, 0.5, 1));
    PlotYmdTooltip(log);
    ImPlot::EndPlot();
}
inline void TabStab(Vehicle &car, UpdateYmd &update_ymd) {
        static field<LogYmd> log = VisualYmdCV(car, update_ymd.refines, update_ymd.v, update_ymd.T, update_ymd.yaw_range, update_ymd.steer_range, update_ymd.yaw_ct, update_ymd.steer_ct);
        if (update_ymd.is_true) {
            log = VisualYmdCV(car, update_ymd.refines, update_ymd.v, update_ymd.T, update_ymd.yaw_range, update_ymd.steer_range, update_ymd.yaw_ct, update_ymd.steer_ct);
            update_ymd.is_true = false;
            //cout << "DEBUG SLIP:\n" << log(floor(update_ymd.yaw_range/2), floor(update_ymd.yaw_range/2)+1).slip << endl;
            //cout << "DEBUG FXT:\n" << log(floor(update_ymd.yaw_range/2), floor(update_ymd.yaw_range/2)+1).fxt << endl;
            //cout << "DEBUG FYT:\n" << log(floor(update_ymd.yaw_range/2), floor(update_ymd.yaw_range/2)+1).fyt << endl;
        }
        ImPlot::BeginSubplots("##Subplots", 2, 2, ImGui::GetContentRegionAvail());
        // Yaw Plot
        ImPlot::BeginPlot("##Yaw", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y/2));
        ImPlot::SetupAxis(ImAxis_X1, "Yaw (deg)", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxis(ImAxis_Y1, "Yaw Accel (rad/s-2)", ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -30.0, 30.0, ImGuiCond_FirstUseEver);
        PlotYawLines(log, ImVec4(0, 0, 1, 1), ImVec4(0, 1, 0.5, 1));
        ImPlot::EndPlot();
        // Steer Plot
        ImPlot::BeginPlot("##Steer", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y));
        ImPlot::SetupAxis(ImAxis_X1, "Steer (deg)", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxis(ImAxis_Y1, "Yaw Accel (rad/s-2)", ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -30.0, 30.0, ImGuiCond_FirstUseEver);
        PlotSteerLines(log, ImVec4(1, 0, 0, 1), ImVec4(0.5, 1, 0, 1));
        ImPlot::EndPlot();
        // Stability Plot
        ImPlot::BeginPlot("##Stability", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y/2));
        ImPlot::SetupAxis(ImAxis_X1, "Yaw (deg)", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxis(ImAxis_Y1, "Stability (rad/deg.s-2)", ImPlotAxisFlags_Invert);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -6.0, 8.0, ImGuiCond_FirstUseEver);
        PlotStabLines(log, ImVec4(0, 0, 1, 1), ImVec4(0, 1, 0.5, 1));
        ImPlot::EndPlot();
        // Control Plot
        ImPlot::BeginPlot("##Control", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y));
        ImPlot::SetupAxis(ImAxis_X1, "Steer (deg)", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxis(ImAxis_Y1, "Control (rad/deg.s-2)", ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -2.0, 5.0, ImGuiCond_FirstUseEver);
        PlotContLines(log, ImVec4(1, 0, 0, 1), ImVec4(0.5, 1, 0, 1));
        ImPlot::EndPlot();
        ImPlot::EndSubplots();
}
inline void TabTire(Vehicle &car, UpdateYmd &update_ymd) {
    switch (update_ymd.type_tire) {
        case 0:
            SubplotsTireFx(car);
        break;
        case 1:
            SubplotsTireFy(car);
        break;
        case 2:
            SubplotsTireMz(car);
        break;
        default:
            break;
    }
}
inline void TabLts(Vehicle &car, UpdateYmd &update_ymd) {

}
// Front-end panel areas
inline void LeftPanel(Vehicle &car, UpdateYmd &update_ymd) {
    ImGui::BeginChild("Left Panel", ImVec2(450, ImGui::GetContentRegionAvail().y), true);
    ImGui::BeginTabBar("Right Tabs", ImGuiTabBarFlags_None);
    if(ImGui::BeginTabItem("YMD Plot")){ // Control isolines and dynamic situation
        InterfaceYmd(car, update_ymd);
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Vehicle")){ // For car tuning purposes
        InterfaceCar(car, update_ymd);
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Tires")){ // For tire fitting purposes
        InterfaceTire(car, update_ymd);
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Motor Control")){ // For torque vectoring purposes
        InterfaceTv(car, update_ymd);
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
    ImGui::EndChild();
    ImGui::SameLine(); // Horizontal panel layout
}
inline void RightPanel(Vehicle &car, UpdateYmd &update_ymd) {
    ImGui::BeginChild("Right Panel", ImGui::GetContentRegionAvail(), true);
    ImGui::BeginTabBar("Right Tabs", ImGuiTabBarFlags_None);
    if(ImGui::BeginTabItem("Yaw Moment Diagram")){
        TabYmd(car, update_ymd);
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Stability/Control")){
        TabStab(car, update_ymd);
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Tire Fitter")){
        TabTire(car, update_ymd);
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Lap Time Simulation")){
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
    ImGui::EndChild();
}

#endif //GUI_FRONTEND_PANELS_H
