#include "Renderer.h"

Renderer::Renderer() : highwayVao(sf::VertexArray(sf::Triangles)), normalRoadVao(sf::VertexArray(sf::Lines))
{
}

void Renderer::addLine(Vec2 st, Vec2 ed, sf::Color color)
{
    sf::Vertex v;
    normalRoadVao.append(sf::Vertex(sf::Vector2f(st.x, st.y), color));
    normalRoadVao.append(sf::Vertex(sf::Vector2f(ed.x, ed.y), color));
}

void Renderer::addTri(Vec2 a, Vec2 b, Vec2 c, sf::Color color)
{
    highwayVao.append(sf::Vertex(sf::Vector2f(a.x, a.y), color));
    highwayVao.append(sf::Vertex(sf::Vector2f(b.x, b.y), color));
    highwayVao.append(sf::Vertex(sf::Vector2f(c.x, c.y), color));
}

void Renderer::addRect(Vec2 st, Vec2 ed, float width, sf::Color color)
{
    Vec2 normal = (ed - st).getNormal() * width / 2.f;
    addTri(st + normal, ed + normal, st - normal, color);
    addTri(st - normal, ed - normal, ed + normal, color);
}

void Renderer::renderCity(sf::RenderWindow &window, const CityGen &cg)
{
    updateData(cg);
    render(window);
}

void Renderer::updateData(const CityGen &cg)
{
    highwayVao.clear();
    normalRoadVao.clear();
    for (const auto &cell : cg)
    {
        for (const auto &r : cell.second)
        {
            if (r->getType() == HighWay)
            {
                addRect(r->getSt(), r->getEd(), 5.f, sf::Color(9, 21, 39));
            }
            else
            {
                addRect(r->getSt(), r->getEd(), 3.f, sf::Color(139, 165, 193));
            }
        }
    }
}

void Renderer::render(sf::RenderWindow &window) const
{
    window.draw(normalRoadVao);
    window.draw(highwayVao);
}
