#ifndef TYPES_HH
#define TYPES_HH

#include <glm/glm.hpp>
#include <array>

typedef double Real;
typedef glm::tvec3<Real, glm::highp> Vector3D;  //3D floating point vector/point, (x,y,z).

struct Triangle {
    std::array<Vector3D, 3> vertices;
    Vector3D normal;
};

#endif //TYPES_HH
