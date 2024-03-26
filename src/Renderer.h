#include "CityGen.h"
#include "Math.h"
#include "RoadNetwork.h"
#include <SFML/Graphics.hpp>

class Renderer
{
  private:
    sf::VertexBuffer roadVbo;
    sf::VertexArray roadVao;

  public:
    Renderer();
    void addLine(Vec2 st, Vec2 ed);
    void updateData(RoadNetwork &rn);
    void render(sf::RenderWindow &window);
    void renderCity(sf::RenderWindow &window, const CityGen &cg);
};
