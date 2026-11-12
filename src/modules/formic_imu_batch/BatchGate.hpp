/****************************************************************************
 * Whether formic_imu_batch publishes its batches.
 *
 *     boot                       the default (FORMIC_BATCH_DEF)
 *     command enable / disable   on / off
 *     disarm (armed -> disarmed) back to the default
 *     arm                        unchanged
 *     the default changes        the new default, also over an earlier command
 *
 * Off until the default is first set. A change of state is reported once by takeChange(): the
 * module registers or unregisters its FIFO callbacks and resets its batchers on it.
 *
 * No uORB: tested with gtest.
 ****************************************************************************/

#pragma once

namespace formic_imu_batch
{

class BatchGate
{
public:
	/// What set the present state.
	enum class Source {
		Parameter,
		Command,
		Disarm,
	};

	/// The default state (FORMIC_BATCH_DEF). Applies when first set and whenever its value changes.
	void setDefault(bool on)
	{
		if (!_default_set || (on != _default_on)) {
			_default_set = true;
			_default_on = on;
			set(on, Source::Parameter);
		}
	}

	/// A formic_batch_enable message.
	void command(bool enable)
	{
		set(enable, Source::Command);
	}

	/// The arming state now. The first call only records it.
	void setArmed(bool armed)
	{
		if (_armed_known && _armed && !armed) {
			set(_default_on, Source::Disarm);
		}

		_armed_known = true;
		_armed = armed;
	}

	bool enabled() const { return _enabled; }

	/// What last changed the state.
	Source source() const { return _source; }

	/// True once after the state changed since the last call.
	bool takeChange()
	{
		const bool changed = _changed;
		_changed = false;
		return changed;
	}

	static const char *sourceName(Source source)
	{
		switch (source) {
		case Source::Command: return "command";

		case Source::Disarm: return "disarm";

		case Source::Parameter:
		default: return "parameter";
		}
	}

private:
	void set(bool on, Source source)
	{
		if (on != _enabled) {
			_enabled = on;
			_source = source;
			_changed = !_changed; // off and on again between two takeChange() calls is no change
		}
	}

	bool _enabled{false};
	bool _changed{false};
	Source _source{Source::Parameter};

	bool _default_set{false};
	bool _default_on{false};

	bool _armed_known{false};
	bool _armed{false};
};

} // namespace formic_imu_batch
