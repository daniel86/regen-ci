#ifndef REGEN_SKIRT_QUAD_H_
#define REGEN_SKIRT_QUAD_H_

#include <regen/meshes/primitives/rectangle.h>
#include "../lod/tessellation.h"

namespace regen {
	/**
	 * \brief A quad mesh where boundary vertices are duplicated and pulled down to
	 * form a skirt or curtain around the quad.
	 *
	 * This can be useful for terrain meshes to hide the seams between adjacent
	 * patches of terrain.
	 */
	class SkirtQuad : public Rectangle {
	public:
		/**
		 * @param cfg the mesh configuration.
		 */
		explicit SkirtQuad(const Config &cfg = Config());

		/**
		 * Sets the skirt width.
		 * @param skirtWidth The width of the skirt.
		 */
		void setSkirtSize(float skirtSize);

		const ref_ptr<Mesh>& skirtMesh() { return skirtMesh_; }

		void updateAttributes() override;

	protected:
		ref_ptr<Mesh> skirtMesh_;
		float skirtSize_ = 0.05f;

		void tessellateRectangle(uint32_t lod, Tessellation &t) override;
	};
} // namespace

#endif /* __RECTANGLE_H__ */
