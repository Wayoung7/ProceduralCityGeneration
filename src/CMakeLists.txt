add_executable(ProceduralCityGeneration main.cpp Renderer.cpp RoadSeg.cpp CityGen.cpp Math.cpp Camera.cpp)
target_link_libraries(ProceduralCityGeneration PRIVATE ImGui-SFML::ImGui-SFML)