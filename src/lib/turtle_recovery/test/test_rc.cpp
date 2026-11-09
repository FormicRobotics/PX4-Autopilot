// Focused regression for the real B8 log with timestamp_last_valid == 0.
#include "../TurtleHelpers.hpp"
#include "../TurtleHelpers.hpp"
#include <cassert>
#include <cstdio>
#include <cstdint>
int main() {
    using turtle::rc::healthy;
    const uint64_t now = 664602747;
    const uint64_t sample = 664598155; // 4.592 ms old in the user's log
    assert(healthy(now, sample, 0, false));
    assert(healthy(now, sample, sample, false));
    const uint64_t last_values[2] = {0, sample};
    for (uint64_t last : last_values) {
        assert(!healthy(now, sample, last, true));
        assert(!healthy(now, 0, last, false));
        assert(!healthy(now, now + 1, last, false));
        assert(!healthy(now, now - 300000, last, false));
        assert(!healthy(now, now - 300001, last, false));
        assert(healthy(now, now - 299999, last, false));
    }
    assert(!healthy(now, sample, now + 1, false));
    assert(!healthy(now, sample, now - 300000, false));
    assert(!healthy(now, sample, now - 300001, false));
    assert(healthy(now, sample, now - 299999, false));
    assert(!healthy(0, 0, 0, false));
    assert(!healthy(0, 1, 0, false));
    assert(!healthy(1000, UINT64_MAX, 0, false));
    const float channels[18] = {0,0,-1,0,-1,-1,-1,-1,-1,-1,-1,.06599f,-1,-1,-1,-1,0,0};
    assert(turtle::rc::select(channels,16,9) < -.99f);
    assert(!std::isfinite(turtle::rc::select(channels,8,9)));
    assert(!std::isfinite(turtle::rc::select(nullptr,16,9)));
    const int8_t functions[29] = {2,0,1,3,7,-1,-1,-1,8,-1,-1,-1,-1,-1,-1,-1,-1,9,-1,-1,4,-1,-1,-1,-1,-1,-1,-1,-1};
    assert(!turtle::rc_conflict(functions,29,9));
    assert(turtle::rc_conflict(functions,29,10)); // kill switch
    assert(turtle::rc_conflict(functions,29,5)); // arm switch
    std::puts("PASS: real-log RC timestamp, missing optional stamp, timeout/future/loss rejection, channel and conflict checks");
}
