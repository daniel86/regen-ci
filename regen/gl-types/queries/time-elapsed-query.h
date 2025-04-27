#ifndef REGEN_TIME_ELAPSED_QUERY_H_
#define REGEN_TIME_ELAPSED_QUERY_H_

#include <regen/gl-types/gl-query.h>

namespace regen {
	class TimeElapsedQuery : public GLQuery<float> {
	public:
		TimeElapsedQuery() : GLQuery<float>(GL_TIME_ELAPSED) {}

	protected:
		float readQueryResult() const override {
			GLuint64 elapsedNano;
			glGetQueryObjectui64v(id(), GL_QUERY_RESULT, &elapsedNano);
			return elapsedNano * 1e-6f; // Convert nanoseconds to milliseconds
		}
	};
} // namespace

#endif /* REGEN_TIME_ELAPSED_QUERY_H_ */
