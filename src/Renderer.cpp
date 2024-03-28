#include "Renderer.h"

Renderer::Renderer() : normalRoadVao(sf::VertexArray(sf::Lines)), highwayVao(sf::VertexArray(sf::Triangles))
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
    normalRoadVao.clear();
    highwayVao.clear();
    for (const auto &cell : cg)
    {
        for (const auto &r : cell.second)
        {
            if (r->getType() == NormalRoad)
            {
                addRect(r->getSt(), r->getEd(), 2.f, sf::Color(185, 197, 211));
            }
            else
            {
                addRect(r->getSt(), r->getEd(), 3.f, sf::Color(139, 165, 193));
            }
        }
    }
    // for (const RoadSeg *r : cg)
    // {
    //     if (r->getType() == NormalRoad)
    //     {
    //         addRect(r->getSt(), r->getEd(), 2.f, sf::Color(185, 197, 211));
    //     }
    //     else
    //     {
    //         // sf::Vector2f st(r->getSt().x, r->getSt().y);
    //         // sf::Vector2f ed(r->getEd().x, r->getEd().y);
    //         // sf::Vector2f mid((st + ed) / 2.f);
    //         // sf::RectangleShape highway(sf::Vector2f(r->getLen() + 1.f, 2.f));
    //         // highway.rotate(r->getDir() / M_PI * 180.f);
    //         // highway.setPosition(mid);
    //         // window.draw(highway);
    //         addRect(r->getSt(), r->getEd(), 3.f, sf::Color(139, 165, 193));
    //     }
    //     // if (r->getStNeighbors().size() > 2)
    //     // {
    //     //     sf::RectangleShape crossing(sf::Vector2f(5.f, 5.f));
    //     //     crossing.setPosition(sf::Vector2f(r->getSt().x, r->getEd().y));
    //     //     crossing.setFillColor(sf::Color(255, 0, 0));
    //     //     window.draw(crossing);
    //     // }

    //     // if (r->getEdNeighbors().size() > 2)
    //     // {
    //     //     sf::RectangleShape crossing(sf::Vector2f(5.f, 5.f));
    //     //     crossing.setPosition(sf::Vector2f(r->getEd().x, r->getEd().y));
    //     //     crossing.setFillColor(sf::Color(255, 0, 0));
    //     //     window.draw(crossing);
    //     // }
    //     // for (RoadSeg *stNeighbor : r->getStNeighbors())
    //     // {
    //     //     sf::RectangleShape crossingSt(sf::Vector2f(6.f, 6.f));
    //     //     crossingSt.setPosition(sf::Vector2f(stNeighbor->getEd().x, stNeighbor->getEd().y));
    //     //     crossingSt.setFillColor(sf::Color(255, 0, 0));
    //     //     window.draw(crossingSt);
    //     // }
    //     // for (RoadSeg *edNeighbor : r->getEdNeighbors())
    //     // {
    //     //     sf::RectangleShape crossingEd(sf::Vector2f(6.f, 6.f));
    //     //     crossingEd.setPosition(sf::Vector2f(edNeighbor->getSt().x, edNeighbor->getSt().y));
    //     //     crossingEd.setFillColor(sf::Color(255, 0, 0));
    //     //     window.draw(crossingEd);
    //     // }
    // }
    window.draw(normalRoadVao);
    window.draw(highwayVao);
}
