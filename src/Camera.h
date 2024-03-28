#include <SFML/Graphics.hpp>

class Camera
{
  private:
    sf::View view;
    sf::Vector2f lastMousePos;
    float zoom;

  public:
    Camera(sf::RenderWindow &window);
    void handleMouseDrag(sf::RenderWindow &window);
    void handleZoom(sf::RenderWindow &window, sf::Event event);
    inline const sf::View &getView() const
    {
        return view;
    }
};
