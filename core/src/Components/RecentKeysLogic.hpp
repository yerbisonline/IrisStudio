#pragma once

#include "Types/GameState.hpp"

// When a demo's saved keys are read after it loads, apart from the game so it can
// be tested. A load is asked on the main thread as the demo command runs, before the game has loaded anything, and
// through most of a load the game still says it's in a demo, as it only notices the old one has gone in its next frames.
// Reading at the first frame "in a demo" read the keys during the load and again once the demo played, and the keys
// doubled. So each load is read once, after the game was seen out of the old demo (reading the new one, or at the
// menu) and back in one; while a load waits for its read, nothing is written, so the cleared keys never overwrite the
// file of the demo that's loading.
namespace IWXMVM::Components::RecentKeysLogic
{
    // milliseconds a load waits to see the game leave the old demo before its keys are read anyway
    inline constexpr std::uint64_t READ_ANYWAY_AFTER = 60000;

    struct Loads
    {
        std::uint32_t asked = 0;  // loads seen asked so far
        std::uint32_t read = 0;   // the last of them whose keys were read
        bool leftDemo = false;    // the game was out of a demo since the last load was asked
        std::uint64_t askedAt = 0;
    };

    // what the game does in a frame
    struct Seen
    {
        std::uint64_t now = 0;
        std::uint32_t asked = 0;  // loads asked so far (the main thread counts them)
        Types::GameState state = Types::GameState::MainMenu;
        // whether it reads a demo it was asked to play; none when the game can't tell, which reads at the first frame
        // in a demo, as before
        std::optional<bool> reading;
    };

    // a frame: true when the keys are to be read now (the load counted as read)
    bool Advance(Loads& loads, const Seen& seen);
    // a load waits for its keys: they aren't written meanwhile
    bool IsReadPending(const Loads& loads);
}  // namespace IWXMVM::Components::RecentKeysLogic
