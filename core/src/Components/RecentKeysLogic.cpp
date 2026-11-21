#include "StdInclude.hpp"
#include "RecentKeysLogic.hpp"

namespace IWXMVM::Components::RecentKeysLogic
{
    bool Advance(Loads& loads, const Seen& seen)
    {
        if (seen.asked != loads.asked)
        {
            loads.asked = seen.asked;
            loads.leftDemo = false;
            loads.askedAt = seen.now;
        }
        if (!IsReadPending(loads))
            return false;
        if (seen.state != Types::GameState::InDemo)
        {
            loads.leftDemo = true;
            return false;
        }
        if (seen.reading && !loads.leftDemo && seen.now < loads.askedAt + READ_ANYWAY_AFTER)
            return false;
        loads.read = loads.asked;
        return true;
    }

    bool IsReadPending(const Loads& loads)
    {
        return loads.read != loads.asked;
    }
}  // namespace IWXMVM::Components::RecentKeysLogic
