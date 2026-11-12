/****************************************************************************
 * The anchor of an estimate: a rotation about down and a shift that put it onto another frame.
 *
 *     q_out = Rz(psi) * q        p_out = Rz(psi) * p + t        v_out = Rz(psi) * v
 *
 * formic_vio_est anchors a new VIO epoch onto its own output. The EKF2 selector anchors each
 * estimator instance onto the published pose at a handover.
 ****************************************************************************/

#pragma once

#include <cmath>

#include <matrix/math.hpp>

namespace formic_frames
{

/// Heading as PX4 publishes it: the Euler yaw.
inline float yaw_of(const matrix::Quatf &q)
{
	return matrix::Eulerf(q).psi();
}

/// Rotation about down by `psi`.
inline matrix::Quatf rotation_about_down(float psi)
{
	return matrix::Quatf(matrix::AxisAnglef(matrix::Vector3f(0.f, 0.f, 1.f), psi));
}

struct Anchor {
	float psi{0.f};          ///< rotation about down, rad
	matrix::Vector3f t{};    ///< shift NED, m

	matrix::Quatf rotation() const { return rotation_about_down(psi); }

	matrix::Quatf apply(const matrix::Quatf &q) const { return (rotation() * q).normalized(); }

	matrix::Vector3f applyPosition(const matrix::Vector3f &p) const { return matrix::Dcmf(rotation()) * p + t; }

	matrix::Vector3f applyVector(const matrix::Vector3f &v) const { return matrix::Dcmf(rotation()) * v; }

	matrix::Vector2f applyVector(const matrix::Vector2f &v) const
	{
		const float c = cosf(psi);
		const float s = sinf(psi);
		return matrix::Vector2f(c * v(0) - s * v(1), s * v(0) + c * v(1));
	}

	float applyHeading(float heading) const { return matrix::wrap_pi(heading + psi); }

	/// A reset delta of the instance's attitude, seen in the anchored frame: Rz * dq * Rz^-1.
	matrix::Quatf applyDeltaQ(const matrix::Quatf &dq) const
	{
		const matrix::Quatf r = rotation();
		return (r * dq * r.inversed()).normalized();
	}

	/// Maps `v` in the anchored frame back into the instance's frame.
	matrix::Vector2f unapplyVector(const matrix::Vector2f &v) const
	{
		const Anchor inverse{-psi, {}};
		return inverse.applyVector(v);
	}

	/**
	 * The anchor that puts the source pose onto the target pose: the same heading and position.
	 * Both poses must be at the same time.
	 */
	static Anchor between(const matrix::Quatf &q_target, const matrix::Vector3f &p_target,
			      const matrix::Quatf &q_source, const matrix::Vector3f &p_source)
	{
		Anchor a;
		a.psi = matrix::wrap_pi(yaw_of(q_target) - yaw_of(q_source));
		a.t = p_target - matrix::Dcmf(a.rotation()) * p_source;
		return a;
	}
};

/**
 * Offset added to the new instance's anchored unaided heading after a handover, so that the published
 * heading - unaided_heading changes only by the announced heading reset. The stick yaw (StickYaw) takes any
 * other change of it for a converging yaw estimate and turns the yaw setpoint by it.
 * @param pub_heading, pub_unaided    published heading and unaided heading before the handover, rad
 * @param inst_heading, inst_unaided  the new instance's own heading and unaided heading, rad
 * @param announced_delta             heading reset announced with the handover, rad (0 when none)
 * @return the offset, rad; 0 when a value is not finite
 */
inline float unaided_offset(float pub_heading, float pub_unaided, float inst_heading, float inst_unaided,
			    float announced_delta)
{
	if (!std::isfinite(pub_heading) || !std::isfinite(pub_unaided) || !std::isfinite(inst_heading)
	    || !std::isfinite(inst_unaided)) {
		return 0.f;
	}

	// published after: heading_pub' - unaided_pub' = (pub_heading - pub_unaided) + announced_delta, where both
	// primes carry the same anchor rotation, so the rotation cancels
	return matrix::wrap_pi((inst_heading - inst_unaided) - (pub_heading - pub_unaided) - announced_delta);
}

/**
 * The tilt between two attitudes: the north and east parts of the rotation vector of
 * q_target * q_source^-1, in the earth frame. The down part (heading) is dropped.
 */
inline matrix::Vector3f tilt_offset(const matrix::Quatf &q_target, const matrix::Quatf &q_source)
{
	matrix::Quatf dq = (q_target * q_source.inversed()).normalized();

	if (dq(0) < 0.f) {
		dq = matrix::Quatf(-dq(0), -dq(1), -dq(2), -dq(3));
	}

	const matrix::Vector3f rotvec = matrix::AxisAnglef(dq);
	return matrix::Vector3f(rotvec(0), rotvec(1), 0.f);
}

/// Rotates `q` by the earth-frame rotation vector `d_theta * scale`.
inline matrix::Quatf apply_tilt(const matrix::Vector3f &d_theta, float scale, const matrix::Quatf &q)
{
	return (matrix::Quatf(matrix::AxisAnglef(d_theta * scale)) * q).normalized();
}

/// The jump of a published estimate that changes from one value to another at one time.
struct Step {
	matrix::Vector3f delta_p{};
	matrix::Vector3f delta_v{};
	float delta_heading{0.f};
	matrix::Quatf delta_q{};   ///< q_to = delta_q * q_from
};

inline Step step_between(const matrix::Quatf &q_from, const matrix::Vector3f &p_from, const matrix::Vector3f &v_from,
			 const matrix::Quatf &q_to, const matrix::Vector3f &p_to, const matrix::Vector3f &v_to)
{
	Step s;
	s.delta_p = p_to - p_from;
	s.delta_v = v_to - v_from;
	s.delta_heading = matrix::wrap_pi(yaw_of(q_to) - yaw_of(q_from));
	s.delta_q = (q_to * q_from.inversed()).normalized();
	return s;
}

/// Weight of the tilt ramp `dt_s` after the handover: exp(-dt / tau). 0 for tau <= 0.
inline float ramp_scale(float dt_s, float tau_s)
{
	if (!(tau_s > 0.f)) {
		return 0.f;
	}

	return expf(-fmaxf(dt_s, 0.f) / tau_s);
}

} // namespace formic_frames
