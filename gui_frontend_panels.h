//
// Created by benjmyn on 3/29/25.
//

#ifndef GUI_FRONTEND_PANELS_H
#define GUI_FRONTEND_PANELS_H
#include "diff.h"
#include "Log.h"
#include "VisualYmd.h"

struct UpdateYmd {
    bool is_true = false;
    // Needs floats for sliders
    float v = 20;
    float R = 100;
    float T = 0;
    float yaw_range = 6;
    float steer_range = 10;
    int yaw_ct = 55;
    int steer_ct = 55;
    int refines = 7;
};

inline void PlotYmdTooltip(const field<LogYmd> &log) {
    for (int i = 0; i < log.n_rows; ++i) {
        for (int j = 0; j < log.n_cols; ++j) {
            ImVec2 point_pos = ImPlot::PlotToPixels(ImPlotPoint(log(i, j).ay, log(i, j).aa));
            ImVec2 mouse_pos = ImPlot::PlotToPixels(ImPlot::GetPlotMousePos());
            double dx = mouse_pos.x - point_pos.x;
            double dy = mouse_pos.y - point_pos.y;
            if (sqrt(dx*dx+dy*dy) < 5) {
                ImGui::SetTooltip("ax = %.2fm.s-2\nay = %.2fm.s-2\naa = %.2frad.s-2\n"
                            "slip = \n %+.1f° %+.1f°\n %+.1f° %+.1f°\n"
                            "Tw = \n %+.0f %+.0f [Nm]\n %+.0f %+.0f [Nm]\n"
                            "fx = \n %+.0f %+.0f [N]\n %+.0f %+.0f [N]\n"
                            "fy = \n %+.0f %+.0f [N]\n %+.0f %+.0f [N]\n"
                            "fz = \n %+.0f %+.0f [N]\n %+.0f %+.0f [N]\n"
                            "yaw = %.1f\n"
                            "steer = %.1f\n"
                            "fxflags =\n %u %u\n %u %u",
                            log(i, j).ax, log(i,j).ay, log(i,j).aa,
                            log(i, j).slip(0), log(i, j).slip(1), log(i, j).slip(2), log(i, j).slip(3),
                            log(i, j).Tw(0), log(i, j).Tw(1), log(i, j).Tw(2), log(i, j).Tw(3),
                            log(i, j).fxt(0), log(i, j).fxt(1), log(i, j).fxt(2), log(i, j).fxt(3),
                            log(i, j).fyt(0), log(i, j).fyt(1), log(i, j).fyt(2), log(i, j).fyt(3),
                            log(i, j).fz(0), log(i, j).fz(1), log(i, j).fz(2), log(i, j).fz(3),
                            log(i,j).yaw, log(i, j).steer,
                            log(i, j).fxflags(0), log(i, j).fxflags(1), log(i, j).fxflags(2), log(i, j).fxflags(3)
                            );
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
inline void LeftPanel(Vehicle &car, UpdateYmd &update_ymd) {
    ImGui::BeginChild("Left Panel", ImVec2(450, ImGui::GetContentRegionAvail().y), true);
    ImGui::BeginTabBar("Right Tabs", ImGuiTabBarFlags_None);
    if(ImGui::BeginTabItem("Title Card")){
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
	    ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Vehicle")){
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Tires")){
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Motor Control")){
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("YMD Inputs")){
        ImGui::SeparatorText("Yaw Moment Diagram Control");
        if (ImGui::Button("Update YMD")){update_ymd.is_true = true;}
        ImGui::PushItemWidth(212);
        ImGui::SliderFloat("##slider1", &update_ymd.v, 10, 30, "v = %.0f");
        ImGui::SameLine();
        ImGui::SliderFloat("##slider2", &update_ymd.T, -1000, 800, "T = %.0f");
        ImGui::SliderInt("##slider3", &update_ymd.yaw_ct, 10, 100, "yaw isolines = %i");
        ImGui::SameLine();
        ImGui::SliderInt("##slider4", &update_ymd.steer_ct, 10, 100, "steer isolines = %i");
        ImGui::SliderFloat("##slider5", &update_ymd.yaw_range, 0, 15, "yaw range = %.0f");
        ImGui::SameLine();
        ImGui::SliderFloat("##slider6", &update_ymd.steer_range, 0, 25, "steer range = %.0f");
        ImGui::SliderInt("##slider7", &update_ymd.refines, 5, 40, "refines = %i");
        ImGui::PopItemWidth();
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
    ImGui::EndChild();
    ImGui::SameLine();
}
inline void RightPanel(Vehicle &car, UpdateYmd &update_ymd) {
    ImGui::BeginChild("Right Panel", ImGui::GetContentRegionAvail(), true);
    ImGui::BeginTabBar("Right Tabs", ImGuiTabBarFlags_None);
    if(ImGui::BeginTabItem("Tire Fitter")){
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Yaw Moment Diagram")){
        ImPlot::BeginPlot("Yaw Moment Diagram", ImGui::GetContentRegionAvail());
        ImPlot::SetupAxis(ImAxis_X1, "Lateral Accel (m.s-2)", ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_X1, -20.0, 20.0, ImGuiCond_FirstUseEver);
        ImPlot::SetupAxis(ImAxis_Y1, "Yaw Accel (rad.s-2)", ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -25.0, 25.0, ImGuiCond_FirstUseEver);
        // YMD
        static field<LogYmd> log = VisualYmdCV(car, update_ymd.refines, update_ymd.v, update_ymd.T, update_ymd.yaw_range, update_ymd.steer_range, update_ymd.yaw_ct, update_ymd.steer_ct);
        if (update_ymd.is_true) {
            log = VisualYmdCV(car, update_ymd.refines, update_ymd.v, update_ymd.T, update_ymd.yaw_range, update_ymd.steer_range, update_ymd.yaw_ct, update_ymd.steer_ct);
            update_ymd.is_true = false;
            //cout << "DEBUG SLIP:\n" << log(floor(update_ymd.yaw_range/2), floor(update_ymd.yaw_range/2)+1).slip << endl;
            //cout << "DEBUG FXT:\n" << log(floor(update_ymd.yaw_range/2), floor(update_ymd.yaw_range/2)+1).fxt << endl;
            //cout << "DEBUG FYT:\n" << log(floor(update_ymd.yaw_range/2), floor(update_ymd.yaw_range/2)+1).fyt << endl;
        }
        // YMD
        PlotYmdLines(log, ImVec4(1, 0, 0, 1), ImVec4(0.5, 1, 0, 1),
                            ImVec4(0, 0, 1, 1), ImVec4(0, 1, 0.5, 1));
        PlotYmdTooltip(log);
        ImPlot::EndPlot();
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Stability/Control")){
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
        ImPlot::SetupAxis(ImAxis_Y1, "Stability (rad/deg.s-2)", ImPlotAxisFlags_None);
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
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Lap Time Simulation")){
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
    ImGui::EndChild();
}

#endif //GUI_FRONTEND_PANELS_H
