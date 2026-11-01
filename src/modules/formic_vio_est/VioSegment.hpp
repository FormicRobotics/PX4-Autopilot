/****************************************************************************
 * VIO segments: the stretches during which the output frame of formic_vio_est stays continuous.
 *
 * Each VIO epoch (restart) has a new, arbitrary frame. A restart after a gap shorter than
 * FORMIC_VIO_GAP_MAX keeps the segment: the new epoch is anchored onto the module's own output,
 * which the IMU carried through the gap. A longer gap starts a new segment, in the frame of the
 * epoch that ends it.
 *
 * No uORB: tested with gtest.
 ****************************************************************************/

#pragma once

#include <cstdint>

#include <lib/formic_frames/anchor.hpp>

namespace formic_vio_est
{

enum class SegmentAction : uint8_t {
	NEW_SEGMENT,  ///< start the output from the pose; the frame is the epoch's
	NEW_EPOCH,    ///< anchor the new epoch onto the output, then correct as usual
	CONTINUE,     ///< correct the output with the pose in the segment frame
};

class VioSegment
{
public:
	bool active() const { return _active; }

	/// Increments with every new segment. 0: none yet.
	uint32_t id() const { return _id; }

	uint64_t epoch() const { return _epoch; }

	/// Epochs anchored onto the output within this segment.
	uint32_t epochChanges() const { return _epoch_changes; }

	/// Maps a pose of the current epoch into the segment frame.
	const formic_frames::Anchor &epochAnchor() const { return _epoch_anchor; }

	/// Sample time of the newest pose used in this segment.
	uint64_t lastSampleTime() const { return _last_sample_us; }

	/**
	 * What to do with a pose that passed every check.
	 * @param gap_max_us longest time between two poses of one segment
	 */
	SegmentAction classify(uint64_t epoch, uint64_t sample_us, uint64_t gap_max_us) const
	{
		if (!_active) {
			return SegmentAction::NEW_SEGMENT;
		}

		const bool long_gap = (sample_us > _last_sample_us) && (sample_us - _last_sample_us > gap_max_us);

		if (long_gap) {
			return SegmentAction::NEW_SEGMENT;
		}

		if (epoch != _epoch) {
			return SegmentAction::NEW_EPOCH;
		}

		return SegmentAction::CONTINUE;
	}

	/// The output was started from a pose of `epoch`: a new segment in that epoch's frame.
	void startSegment(uint64_t epoch, uint64_t sample_us)
	{
		_active = true;
		_id++;
		_epoch = epoch;
		_epoch_changes = 0;
		_epoch_anchor = formic_frames::Anchor{};
		_last_sample_us = sample_us;
	}

	/**
	 * A new epoch within the segment. The pose (q, p) of the new epoch and the output (q_out,
	 * p_out) at its time give the anchor of the epoch.
	 */
	void anchorEpoch(uint64_t epoch, uint64_t sample_us, const matrix::Quatf &q_out, const matrix::Vector3f &p_out,
			 const matrix::Quatf &q, const matrix::Vector3f &p)
	{
		_epoch = epoch;
		_epoch_changes++;
		_epoch_anchor = formic_frames::Anchor::between(q_out, p_out, q, p);
		_last_sample_us = sample_us;
	}

	/// A pose of the current epoch was used.
	void used(uint64_t sample_us)
	{
		if (sample_us > _last_sample_us) {
			_last_sample_us = sample_us;
		}
	}

	/// The output is not usable any more (FORMIC_VIO_GAP_MAX passed): the next pose starts a segment.
	void end() { _active = false; }

private:
	bool _active{false};
	uint32_t _id{0};
	uint64_t _epoch{0};
	uint32_t _epoch_changes{0};
	formic_frames::Anchor _epoch_anchor{};
	uint64_t _last_sample_us{0};
};

} // namespace formic_vio_est
