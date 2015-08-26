#include "obstacle.h"
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>

Obstacle::Obstacle(Vector2D center, float radius, const char *details_)
{
    shape = new b2CircleShape;
    b2CircleShape* circle = (b2CircleShape*) shape;
    circle->m_p.SetZero();
    shape->m_radius = radius;

    transform.Set(center.toB2vec2(), 0);

    repulseStrenght = 1.0;
    speed.setZero();
    if(details_ != 0)
        detail_text.assign( details_ );
}

Obstacle::Obstacle(Vector2D center, float width, float height, float orien, const char *details_)
{
    shape = new b2PolygonShape;
    shape->m_type = b2Shape::e_polygon;    

    ((b2PolygonShape*)shape)->SetAsBox(width/2, height/2); // , center, orien);
    transform.Set(center.toB2vec2(), orien);

    repulseStrenght = 1.0;
    speed.setZero();
    if(details_ != 0)
        detail_text.assign( details_ );
}

b2Transform Obstacle::predictedTransform(float time_step_sec) const
{
    b2Transform t;
    Vector2D new_position = Vector2D(transform.p) + speed * time_step_sec;
    t.Set(new_position.toB2vec2(), transform.q.GetAngle());
    return t;
}

Vector2D Obstacle::CenterOfMass() const
{
    Vector2D COM;
    b2PolygonShape *poly;
    switch(shape->GetType()) {
    case b2Shape::e_circle :
        COM = Vector2D(transform.p);
        break;
    case b2Shape::e_polygon:
        poly = dynamic_cast<b2PolygonShape*>(shape);
        for(int i=0; i<poly->GetVertexCount(); i++)  {
            COM += Vector2D(poly->GetVertex(i));
        }
        COM /= poly->GetVertexCount();
        break;
    default:
        COM = Vector2D(transform.p);
    }
    return COM;

}
