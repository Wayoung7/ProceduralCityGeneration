#include <SFML/Graphics.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>

#include <imgui-SFML.h>
#include <imgui.h>

#include "CityGen.h"
#include "Generator.h"
#include "GlobalConfig.h"
#include "GlobalData.h"
#include "Math.h"
#include "Renderer.h"
#include "RoadNetwork.h"
#include "RoadSeg.h"
#include "RoadSegment.h"

int main()
{
    RNG rng;
    sf::RenderWindow window(sf::VideoMode(2560, 1440), "ImGui + SFML = <3");
    // window.setFramerateLimit(60);
    ImGui::SFML::Init(window);

    // Init
    // vector<RoadSegmentProperty> segments;
    // vector<pair<int, int>> crossings;
    // vector<WaitingSeg> q = {WaitingSeg(
    //     0.f, RoadSegmentProperty(pair(640, 360), pair(650, 360), 1))};

    // GlobalConfig::getInstance().maxRoadCross = 4;
    // GlobalConfig::getInstance().maxSegLen = 30.f;
    // GlobalConfig::getInstance().minSegLen = 20.f;
    GlobalConfig::getInstance().segLimit = 8000;
    GlobalConfig::getInstance().windowWidth = 2560;
    GlobalConfig::getInstance().windowHeight = 1440;

    // GlobalData::getInstance().m_roadSegs = 0;

    // RoadNetwork rn;
    Renderer renderer;
    // Generator g;
    // g.init(rn);
    CityGen cg;
    cg.init();

    sf::Clock deltaClock;
    while (window.isOpen() && GlobalData::getInstance().isRunning)
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(event);

            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
        }

        // if (rn.verticesCount() < 100)
        // {
        //     g.step(rn);
        // }
        cg.step();

        // // Update
        // // Pop smallest t
        // int smallest_idx = 0;
        // int smallest_t = INT_MAX;
        // for (int i = 0; i < q.size(); i++) {
        //     if (q[i].t < smallest_t) {
        //         smallest_idx = i;
        //         smallest_t = q[i].t;
        //     }
        // }
        // WaitingSeg cur = q[smallest_idx];
        // q.erase(q.begin() + smallest_idx);

        // sf::VertexArray a(sf::Lines);
        // a.append(sf::Vertex(sf::Vector2f(0.f, 0.f)));
        // a.append(sf::Vertex(sf::Vector2f(500.f, 500.f)));

        ImGui::SFML::Update(window, deltaClock.restart());

        ImGui::ShowDemoWindow();

        window.clear();

        // renderer.updateData(rn);
        // renderer.render(window);
        // window.draw(a);
        renderer.renderCity(window, cg);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();

    return 0;
}