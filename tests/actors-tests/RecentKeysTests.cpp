#include "StdInclude.hpp"
#include "Test.hpp"

#include "Components/RecentKeysLogic.hpp"

using namespace IWXMVM::Components;
using IWXMVM::Types::GameState;

// A demo's saved keys read once per load, after the new demo plays, and not written while a load waits for them

namespace
{
    RecentKeysLogic::Seen InDemo(std::uint64_t now, std::uint32_t asked, std::optional<bool> reading = false)
    {
        return {.now = now, .asked = asked, .state = GameState::InDemo, .reading = reading};
    }

    RecentKeysLogic::Seen Reading(std::uint64_t now, std::uint32_t asked)
    {
        return {.now = now, .asked = asked, .state = GameState::MainMenu, .reading = true};
    }
}  // namespace

TEST("a demo's keys are read once, after the game leaves the old demo and plays the new one")
{
    RecentKeysLogic::Loads loads;
    CHECK(!RecentKeysLogic::Advance(loads, InDemo(1000, 0)));
    // the demo command of a second demo, while the first plays: the game says it's in a demo for a while yet
    CHECK(!RecentKeysLogic::Advance(loads, InDemo(2000, 1)));
    CHECK(RecentKeysLogic::IsReadPending(loads));
    CHECK(!RecentKeysLogic::Advance(loads, InDemo(4000, 1)));
    // then reads the new one, at the menu
    CHECK(!RecentKeysLogic::Advance(loads, Reading(5000, 1)));
    CHECK(!RecentKeysLogic::Advance(loads, Reading(20000, 1)));
    // and plays it: read once
    CHECK(RecentKeysLogic::Advance(loads, InDemo(23000, 1)));
    CHECK(!RecentKeysLogic::IsReadPending(loads));
    CHECK(!RecentKeysLogic::Advance(loads, InDemo(23016, 1)));
    CHECK(!RecentKeysLogic::Advance(loads, InDemo(60000, 1)));
}

TEST("the first demo of a game, loaded from the menu, is read as it plays")
{
    RecentKeysLogic::Loads loads;
    CHECK(!RecentKeysLogic::Advance(loads, {.now = 1000, .asked = 1, .state = GameState::MainMenu, .reading = false}));
    CHECK(!RecentKeysLogic::Advance(loads, Reading(2000, 1)));
    CHECK(RecentKeysLogic::Advance(loads, InDemo(15000, 1)));
}

TEST("two loads asked before either plays read their keys once, for the last")
{
    RecentKeysLogic::Loads loads;
    CHECK(!RecentKeysLogic::Advance(loads, InDemo(1000, 1)));
    CHECK(!RecentKeysLogic::Advance(loads, Reading(2000, 1)));
    // a second demo command before the first played: it waits for the game to leave a demo again
    CHECK(!RecentKeysLogic::Advance(loads, InDemo(3000, 2)));
    CHECK(!RecentKeysLogic::Advance(loads, Reading(4000, 2)));
    CHECK(RecentKeysLogic::Advance(loads, InDemo(20000, 2)));
    CHECK(!RecentKeysLogic::Advance(loads, InDemo(20016, 2)));
}

TEST("a load the game never shows leaving the demo is read after a minute")
{
    RecentKeysLogic::Loads loads;
    CHECK(!RecentKeysLogic::Advance(loads, InDemo(1000, 1)));
    CHECK(!RecentKeysLogic::Advance(loads, InDemo(1000 + RecentKeysLogic::READ_ANYWAY_AFTER - 1, 1)));
    CHECK(RecentKeysLogic::Advance(loads, InDemo(1000 + RecentKeysLogic::READ_ANYWAY_AFTER, 1)));
}

TEST("a game that can't tell it reads a demo reads the keys at the first frame in a demo, as before")
{
    RecentKeysLogic::Loads loads;
    CHECK(!RecentKeysLogic::Advance(loads, {.now = 900, .asked = 1, .state = GameState::MainMenu}));
    CHECK(RecentKeysLogic::Advance(loads, InDemo(1000, 1, std::nullopt)));
    CHECK(!RecentKeysLogic::Advance(loads, InDemo(1016, 1, std::nullopt)));
}

TEST("keys aren't written while a load waits for its read")
{
    RecentKeysLogic::Loads loads;
    CHECK(!RecentKeysLogic::IsReadPending(loads));
    RecentKeysLogic::Advance(loads, InDemo(1000, 1));
    CHECK(RecentKeysLogic::IsReadPending(loads));
    RecentKeysLogic::Advance(loads, Reading(2000, 1));
    CHECK(RecentKeysLogic::IsReadPending(loads));
    RecentKeysLogic::Advance(loads, InDemo(9000, 1));
    CHECK(!RecentKeysLogic::IsReadPending(loads));
}
