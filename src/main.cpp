#include <SFML/Graphics.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>

#include <imgui-SFML.h>
#include <imgui.h>
#include <iostream>

#include "Camera.h"
#include "CityGen.h"
#include "GlobalConfig.h"
#include "GlobalData.h"
#include "Math.h"
#include "Renderer.h"
#include "RoadSeg.h"

int main()
{
    sf::RenderWindow window(
        sf::VideoMode(GlobalConfig::getInstance().windowWidth, GlobalConfig::getInstance().windowHeight),
        "Procedural City Generation", sf::Style::Default, sf::ContextSettings(0, 0, 2));
    if (!ImGui::SFML::Init(window))
    {
        std::cerr << "Error: Failed to initialize SFML window" << std::endl;
        exit(-1);
    }

    Renderer renderer;
    Camera camera(window);
    CityGen cg;
    cg.init();

    sf::Clock deltaClock;
    sf::Clock timer;
    const auto &io = ImGui::GetIO();
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(event);

            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
            else if (event.type == sf::Event::MouseWheelMoved && !io.WantCaptureMouse)
            {
                camera.handleZoom(window, event);
            }
        }
        if (!io.WantCaptureMouse)
        {
            camera.handleMouseDrag(window);
        }

        cg.step();

        ImGui::SFML::Update(window, deltaClock.restart());

        ImGui::Begin("Debug");
        if (ImGui::Button("Restart"))
        {
            cg.reset();
        }
        if (GlobalData::getInstance().isFinished == false)
        {
            if (ImGui::Button("Stop"))
            {
                cg.stop();
            }
        }
        if (GlobalData::getInstance().isFinished)
        {
            ImGui::BeginChild("Settings");

            ImGui::SliderInt("Segment Limit", &GlobalConfig::getInstance().segLimit, 10, 100000);
            ImGui::SliderInt("Normal Segment Length", &GlobalConfig::getInstance().normalSegLen, 15.f, 35.f);
            ImGui::SliderInt("Highway Segment Length", &GlobalConfig::getInstance().highwaySegLen, 25.f, 45.f);
            ImGui::SliderFloat("Straight Angle Deviation", &GlobalConfig::getInstance().straightAngleDev, 0.f, 0.4f);
            ImGui::SliderFloat("Branch Angle Deviation", &GlobalConfig::getInstance().branchAngleDev, 0.f, 0.4f);
            ImGui::SliderFloat("Normal Branch Probability", &GlobalConfig::getInstance().normalBranchProb, 0.1f, 0.9f);
            ImGui::SliderFloat("Highway Branch Probability", &GlobalConfig::getInstance().highwayBranchProb, 0.0f,
                               0.3f);
            ImGui::SliderInt("Normal Segment Delay", &GlobalConfig::getInstance().normalDelay, 0, 10);
            ImGui::EndChild();
        }
        else
        {
            ImGui::Text("Settings Disabled");
            ImGui::ProgressBar((float)cg.getSegCoung() / (float)GlobalConfig::getInstance().segLimit, ImVec2(300, 20));
            ImGui::Text("Generating: %d/%d", cg.getSegCoung(), GlobalConfig::getInstance().segLimit);
        }
        ImGui::End();

        if (GlobalData::getInstance().isFinished)
        {
            window.setView(camera.getView());
            window.clear(sf::Color(231, 231, 231));
            renderer.renderCity(window, cg);
            timer.restart();
            window.setView(window.getDefaultView());
        }
        else if (timer.getElapsedTime().asSeconds() > 0.2)
        {
            window.setView(camera.getView());
            window.clear(sf::Color(231, 231, 231));
            renderer.renderCity(window, cg);
            timer.restart();
            window.setView(window.getDefaultView());
        }

        ImGui::SFML::Render(window);
        window.display();
    }

    return 0;
}
