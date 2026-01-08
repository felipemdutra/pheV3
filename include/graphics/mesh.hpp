#ifndef MESH_HPP
#define MESH_HPP

#include "physics/mesh.hpp"

namespace phe::graphics {

    void prepareMeshRenderContext(physics::Mesh& mesh);

    void destroyMesh(physics::Mesh& mesh);

    void drawMesh(physics::Mesh& m);

}

#endif

