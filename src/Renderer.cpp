#include "Renderer.h"

Renderer::Renderer()
    : roadVbo(sf::VertexBuffer(sf::Lines, sf::VertexBuffer::Usage::Dynamic)), roadVao(sf::VertexArray(sf::Lines))
{
}

void Renderer::addLine(Vec2 st, Vec2 ed)
{
    // sf::VertexArray lines(sf::Lines, 2);
    // lines[0].position = sf::Vector2f(st.x, st.y);
    // lines[0].color = sf::Color::White;
    // lines[1].position = sf::Vector2f(ed.x, ed.y);
    // lines[1].color = sf::Color::White;
    // roadVbo.update(&lines[0], 2, 0);
    roadVao.append(sf::Vertex(sf::Vector2f(st.x, st.y)));
    roadVao.append(sf::Vertex(sf::Vector2f(ed.x, ed.y)));
}

void Renderer::updateData(RoadNetwork &rn)
{
    for (const auto &v : rn)
    {
        for (const auto &n : v->getNeighbors())
        {
            this->addLine(v->getPos(), n->getPos());
        }
    }
}

void Renderer::render(sf::RenderWindow &window)
{
    window.draw(roadVao);
}

void Renderer::renderCity(sf::RenderWindow &window, const CityGen &cg)
{
    roadVao.clear();
    for (const RoadSeg *r : cg)
    {
        // if (r->getType() == NormalRoad)
        {
            addLine(r->getSt(), r->getEd());
        }
        // else
        // {
        //     sf::Vector2f st(r->getSt().x, r->getSt().y);
        //     sf::Vector2f ed(r->getEd().x, r->getEd().y);
        //     sf::Vector2f mid((st + ed) / 2.f);
        //     sf::RectangleShape highway(sf::Vector2f(r->getLen() + 5.f, 2.f));
        //     highway.setPosition(mid);
        //     highway.rotate(r->getDir() / M_PI * 180.f);
        //     window.draw(highway);
        // }
    }
    window.draw(roadVao);
}
