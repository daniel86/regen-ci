/*
Copyright (c) 2006-2012, assimp team
All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the
following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------
*/

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <map>

#include <regen/math/matrix.h>

namespace regen {
	/**
	 * \brief A number system that extends the complex numbers.
	 *
	 * In computer graphics Quaternion's are used to rotate
	 * using an arbitrary axis.
	 * @see http://en.wikipedia.org/wiki/Quaternion
	 */
	class Quaternion {
	public:
		Quaternion()
				: w(0.0f), x(0.0f), y(0.0f), z(0.0f) {}

		/**
		 * Set-component constructor.
		 */
		Quaternion(GLfloat _w, GLfloat _x, GLfloat _y, GLfloat _z)
				: w(_w), x(_x), y(_y), z(_z) {}

		/**
		 * @param os output stream.
		 * @return string representation.
		 */
		std::ostream &operator<<(std::ostream &os) {
			return os << w << " " << x << " " << y << " " << z;
		}

		/**
		 * @param b another Quaternion.
		 * @return true if each component of other is smaller.
		 */
		inline bool operator<(const Quaternion &b) const {
			return (x < b.x) || (y < b.y) || (z < b.z);
		}

		/**
		 * Check for equality.
		 * @param b another Quaternion.
		 * @return true if equal.
		 */
		inline bool operator==(const Quaternion &b) const {
			return x == b.x && y == b.y && z == b.z && w == b.w;
		}

		/**
		 * Check for inequality.
		 * @param b another Quaternion.
		 * @return true if not equal.
		 */
		inline bool operator!=(const Quaternion &b) const {
			return !(*this == b);
		}

		/**
		 * @param b another Quaternion.
		 * @return the Quaternion product.
		 */
		inline Quaternion operator*(const Quaternion &b) const {
			return {
					w * b.w - x * b.x - y * b.y - z * b.z,
					w * b.x + x * b.w + y * b.z - z * b.y,
					w * b.y + y * b.w + z * b.x - x * b.z,
					w * b.z + z * b.w + x * b.y - y * b.x};
		}

		/**
		 * @param scalar a scalar value.
		 * @return the scaled Quaternion.
		 */
		inline Quaternion operator*(float scalar) const {
			return {w * scalar, x * scalar, y * scalar, z * scalar};
		}

		/**
		 * @param b another Quaternion.
		 * @return the dot product.
		 */
		inline void operator+=(const Quaternion &b) {
			w += b.w;
			x += b.x;
			y += b.y;
			z += b.z;
		}

		/**
		 * Construction from euler angles.
		 * @param fPitch the pith angle.
		 * @param fYaw the yaw angle.
		 * @param fRoll the roll angle.
		 */
		inline void setEuler(float fPitch, float fYaw, float fRoll) {
			const float fSinPitch(sinf(fPitch * 0.5f));
			const float fCosPitch(cosf(fPitch * 0.5f));
			const float fSinYaw(sinf(fYaw * 0.5f));
			const float fCosYaw(cosf(fYaw * 0.5f));
			const float fSinRoll(sinf(fRoll * 0.5f));
			const float fCosRoll(cosf(fRoll * 0.5f));
			const float fCosPitchCosYaw(fCosPitch * fCosYaw);
			const float fSinPitchSinYaw(fSinPitch * fSinYaw);
			x = fSinRoll * fCosPitchCosYaw - fCosRoll * fSinPitchSinYaw;
			y = fCosRoll * fSinPitch * fCosYaw + fSinRoll * fCosPitch * fSinYaw;
			z = fCosRoll * fCosPitch * fSinYaw - fSinRoll * fSinPitch * fCosYaw;
			w = fCosRoll * fCosPitchCosYaw + fSinRoll * fSinPitchSinYaw;
		}

		/**
		 * Construction from an axis-angle pair.
		 * @param axis the rotation axis.
		 * @param angle the rotation angle.
		 */
		inline void setAxisAngle(const Vec3f &axis, GLfloat angle) {
			const float sin_a = sinf(angle / 2);
			const float cos_a = cosf(angle / 2);
			x = axis.x * sin_a;
			y = axis.y * sin_a;
			z = axis.z * sin_a;
			w = cos_a;
		}

		/**
		 * Sets the rotation of this Quaternion to look in a specific direction.
		 * @param direction the direction vector to look at.
		 * @param up the up vector, defaults to (0, 1, 0).
		 */
		inline void setLookRotation(const Vec3f& forward, const Vec3f& up = Vec3f::up()) {
			Vec3f right = up.cross(forward);
			right.normalize();
			Vec3f newUp = forward.cross(right);

			// Construct rotation matrix (column-major)
			float m00 = right.x,  m01 = right.y,  m02 = right.z;
			float m10 = newUp.x,  m11 = newUp.y,  m12 = newUp.z;
			float m20 = forward.x, m21 = forward.y, m22 = forward.z;

			// Convert rotation matrix to quaternion
			float trace = m00 + m11 + m22;
			if (trace > 0.0f) {
				float s = 0.5f / sqrtf(trace + 1.0f);
				w = 0.25f / s;
				x = (m21 - m12) * s;
				y = (m02 - m20) * s;
				z = (m10 - m01) * s;
			} else {
				if (m00 > m11 && m00 > m22) {
					float s = 2.0f * sqrtf(1.0f + m00 - m11 - m22);
					w = (m21 - m12) / s;
					x = 0.25f * s;
					y = (m01 + m10) / s;
					z = (m02 + m20) / s;
				} else if (m11 > m22) {
					float s = 2.0f * sqrtf(1.0f + m11 - m00 - m22);
					w = (m02 - m20) / s;
					x = (m01 + m10) / s;
					y = 0.25f * s;
					z = (m12 + m21) / s;
				} else {
					float s = 2.0f * sqrtf(1.0f + m22 - m00 - m11);
					w = (m10 - m01) / s;
					x = (m02 + m20) / s;
					y = (m12 + m21) / s;
					z = 0.25f * s;
				}
			}
			normalize();
		}

		/**
		 * Construction from am existing, normalized Quaternion.
		 * @param normalized a normalized Quaternion.
		 */
		inline void setQuaternion(const Quaternion &normalized) {
			x = normalized.x;
			y = normalized.y;
			z = normalized.z;

			const float t = 1.0f - (x * x) - (y * y) - (z * z);
			if (t < 0.0f) w = 0.0f;
			else w = sqrtf(t);
		}

		/**
		 * @return a matrix that represents this Quaternion.
		 */
		inline Mat4f calculateMatrix() const {
			Mat4f mat4x4 = Mat4f::identity();
			mat4x4.x[0] = -2.0f * (y * y + z * z) + 1.0f;
			mat4x4.x[1] = 2.0f * (x * y - z * w);
			mat4x4.x[2] = 2.0f * (x * z + y * w);
			mat4x4.x[4] = 2.0f * (x * y + z * w);
			mat4x4.x[5] = -2.0f * (x * x + z * z) + 1.0f;
			mat4x4.x[6] = 2.0f * (y * z - x * w);
			mat4x4.x[8] = 2.0f * (x * z - y * w);
			mat4x4.x[9] = 2.0f * (y * z + x * w);
			mat4x4.x[10] = -2.0f * (x * x + y * y) + 1.0f;
			return mat4x4;
		}

		/**
		 * Performs a spherical interpolation between two quaternions
		 * Implementation adopted from the gmtl project.
		 * @param pStart the start value.
		 * @param pEnd the end value.
		 * @param pFactor the blend factor.
		 */
		inline void interpolate(
				const Quaternion &pStart,
				const Quaternion &pEnd,
				float pFactor) {
			// calc cosine theta
			float cosom =
					pStart.x * pEnd.x +
					pStart.y * pEnd.y +
					pStart.z * pEnd.z +
					pStart.w * pEnd.w;

			// adjust signs (if necessary)
			Quaternion end = pEnd;
			if (cosom < 0.0f) {
				cosom = -cosom;
				end.x = -end.x;   // Reverse all signs
				end.y = -end.y;
				end.z = -end.z;
				end.w = -end.w;
			}

			// Calculate coefficients
			float sclp, sclq;
			if ((1.0f - cosom) > 0.0001f) // 0.0001 -> some epsillon
			{
				// Standard case (slerp)
				float omega, sinom;
				omega = acosf(cosom); // extract theta from dot product's cos theta
				sinom = sinf(omega);
				sclp = sinf((1.0f - pFactor) * omega) / sinom;
				sclq = sinf(pFactor * omega) / sinom;
			} else {
				// Very close, do linear interp (because it's faster)
				sclp = 1.0f - pFactor;
				sclq = pFactor;
			}

			x = sclp * pStart.x + sclq * end.x;
			y = sclp * pStart.y + sclq * end.y;
			z = sclp * pStart.z + sclq * end.z;
			w = sclp * pStart.w + sclq * end.w;
		}

		/**
		 * Performs a linear interpolation between two quaternions
		 * Implementation adopted from the gmtl project.
		 * @param pStart the start value.
		 * @param pEnd the end value.
		 * @param pFactor the blend factor.
		 */
		inline void interpolateLinear(
				const Quaternion &pStart,
				const Quaternion &pEnd,
				float pFactor) {
			// calc cosine theta
			float cosom =
					pStart.x * pEnd.x +
					pStart.y * pEnd.y +
					pStart.z * pEnd.z +
					pStart.w * pEnd.w;

			// adjust signs (if necessary)
			Quaternion end = pEnd;
			if (cosom < 0.0f) {
				cosom = -cosom;
				end.x = -end.x;   // Reverse all signs
				end.y = -end.y;
				end.z = -end.z;
				end.w = -end.w;
			}

			// Very close, do linear interp (because it's faster)
			float sclp = 1.0f - pFactor;
			float sclq = pFactor;

			x = sclp * pStart.x + sclq * end.x;
			y = sclp * pStart.y + sclq * end.y;
			z = sclp * pStart.z + sclq * end.z;
			w = sclp * pStart.w + sclq * end.w;
		}

		/**
		 * Compute the magnitude and divide through it.
		 */
		inline void normalize() {
			const float mag = sqrtf(x * x + y * y + z * z + w * w);
			if (mag) {
				const float invMag = 1.0f / mag;
				x *= invMag;
				y *= invMag;
				z *= invMag;
				w *= invMag;
			}
		}

		/**
		 * Conjugation of quaternions is analogous to conjugation of complex numbers.
		 */
		inline void conjugate() {
			x = -x;
			y = -y;
			z = -z;
		}

		/**
		 * @param v the input vector.
		 * @return the input vector rotated using this Quaternion.
		 */
		inline Vec3f rotate(const Vec3f &v) {
			Quaternion q2(0.0f, v.x, v.y, v.z);
			Quaternion q = *this;
			q.conjugate();
			q = q * q2 * (*this);
			return {q.x, q.y, q.z};
		}

		/** Quaternion w-component. */
		float w;
		/** Quaternion x-component. */
		float x;
		/** Quaternion y-component. */
		float y;
		/** Quaternion z-component. */
		float z;
	};
} // namespace

#endif /* QUATERNION_H_ */
