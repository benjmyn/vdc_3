
#ifndef GUI_H
#define GUI_H

//#define GLFW_INCLUDE_NONE
inline GLFWwindow* graphicsWindow;
inline ImGuiWindowFlags parentFlags;
inline int guiWidth = 1280;
inline int guiHeight = 720;

inline void StartWindow() {
    if (!glfwInit()) {
        exit(EXIT_FAILURE);
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create graphics window
    graphicsWindow = glfwCreateWindow(guiWidth, guiHeight, "Vehicle Dynamics Collection", NULL, NULL);
    if (graphicsWindow == NULL)
        exit(EXIT_FAILURE);
    glfwMakeContextCurrent(graphicsWindow); // Tell opengl to target this window

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        exit(EXIT_FAILURE);

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void) io;

    // Configure font
    ImFontConfig config;
    config.SizePixels = 18.0f; // --- FONT SIZE ---
    io.Fonts->AddFontDefault(&config);
    io.Fonts->Build();

    // Dark mode
    ImGui::StyleColorsDark();

    // Parent window flags
    parentFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;

    /* === ImPlot Setup === */
    ImPlot::CreateContext();
    ImPlot::StyleColorsDark();

    /* === Platform/renderer bindings === */
    const char* glsl_version = "#version 130";
    ImGui_ImplGlfw_InitForOpenGL(graphicsWindow, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

}

inline void EndWindow(){
    // Shut down ImPlot
    ImPlot::DestroyContext();

    // Shut down ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    // Shut down GLFW
    glfwDestroyWindow(graphicsWindow);
    glfwTerminate();
}

inline void NewFrame(){
    // Create new frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Set up parent window
    glfwGetFramebufferSize(graphicsWindow, &guiWidth, &guiHeight);
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once, ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2(static_cast<float>(guiWidth), static_cast<float>(guiHeight)));

    // Start parent window (here for convenience)
    ImGui::Begin("Parent Window", nullptr, parentFlags);
}

inline void RenderFrame(){
    // End parent window (here for convenience)
    ImGui::End();

    // Render frame
    ImGui::Render();

    // OpenGL buffer swap
    glViewport(0, 0, 960, 720);
    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwPollEvents(); // Handle event triggers
    glfwSwapBuffers(graphicsWindow);
}

#endif //GUI_H
