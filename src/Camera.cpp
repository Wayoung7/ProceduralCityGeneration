#include "Camera.h"

Camera::Camera(sf::RenderWindow &window) : view(window.getView()), zoom(1.f)
{
}

void Camera::handleMouseDrag(sf::RenderWindow &window)
{
    if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
    {
        sf::Vector2f mousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        sf::Vector2f delta = lastMousePos - mousePos;
        view.move(delta / zoom);
    }
    lastMousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
}

void Camera::handleZoom(sf::RenderWindow &window, sf::Event event)
{
    float zoomAmount = 0.1f;
    zoom += event.mouseWheel.delta * zoomAmount;
    if (zoom < 0.1f)
    {
        zoom = 0.1f;
    }
    view.setSize(window.getSize().x / zoom, window.getSize().y / zoom);
}
