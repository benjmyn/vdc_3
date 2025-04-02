//
// Created by benjmyn on 3/29/25.
//

#ifndef GUI_COLOR_THEMES_H
#define GUI_COLOR_THEMES_H

namespace ImGui {
    inline void StyleColorsCustomLight(ImGuiStyle* dst = nullptr) {
        ImGui::StyleColorsLight();
        if (dst == nullptr)
            dst = &ImGui::GetStyle();
        //dst->Colors[ImGuiCol_Tab] = ImVec4(0.90f, 0.46f, 0.12f, 0.31f); // Burnt Orange
        //dst->Colors[ImGuiCol_TabHovered] = ImVec4(0.90f, 0.46f, 0.12f, 0.70f); // Burnt Orange
        //dst->Colors[ImGuiCol_TabActive] = ImVec4(0.90f, 0.46f, 0.12f, 0.80f); // Burnt Orange
    }
}

namespace ImGui {
    inline void StyleColorsCustomDark(ImGuiStyle* dst = nullptr) {
        ImGui::StyleColorsDark();
        if (dst == nullptr)
            dst = &ImGui::GetStyle();
        dst->Colors[ImGuiCol_Tab] = ImVec4(0.53f, 0.12f, 0.26f, 0.50f); // Chicago Maroon
        dst->Colors[ImGuiCol_TabHovered] = ImVec4(0.53f, 0.12f, 0.26f, 0.85f); // Chicago Maroon
        dst->Colors[ImGuiCol_TabActive] = ImVec4(0.53f, 0.12f, 0.26f, 1.00f); // Chicago Maroon
        dst->Colors[ImGuiCol_Button] = ImVec4(0.53f, 0.12f, 0.26f, 0.75f); // Chicago Maroon
        dst->Colors[ImGuiCol_ButtonHovered] = ImVec4(0.53f, 0.12f, 0.26f, 0.85f); // Chicago Maroon
        dst->Colors[ImGuiCol_ButtonActive] = ImVec4(0.53f, 0.12f, 0.26f, 1.00f); // Chicago Maroon
        dst->Colors[ImGuiCol_SliderGrab] = dst->Colors[ImGuiCol_ButtonHovered]; // Chicago Maroon
        dst->Colors[ImGuiCol_SliderGrabActive] = dst->Colors[ImGuiCol_ButtonActive]; // Chicago Maroon
        dst->Colors[ImGuiCol_FrameBg] = dst->Colors[ImGuiCol_Tab];
        dst->Colors[ImGuiCol_FrameBgHovered] = dst->Colors[ImGuiCol_TabHovered];
        dst->Colors[ImGuiCol_FrameBgActive] = dst->Colors[ImGuiCol_TabActive];

    }
}

#endif //GUI_COLOR_THEMES_H
