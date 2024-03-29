#include "CityGen.h"
#include "Math.h"
#include <SFML/Graphics.hpp>

class Renderer
{
  private:
    sf::VertexArray normalRoadVao;
    sf::VertexArray highwayVao;

    void addLine(Vec2 st, Vec2 ed, sf::Color color);
    void addTri(Vec2 a, Vec2 b, Vec2 c, sf::Color color);
    void addRect(Vec2 st, Vec2 ed, float width, sf::Color color);

  public:
    Renderer();
    void renderCity(sf::RenderWindow &window, const CityGen &cg);
    void updateData(const CityGen &cg);
    void render(sf::RenderWindow &window) const;
};
