#pragma once

namespace IWXMVM::Components
{
    namespace KeyframeSerializer
    {
        void Write(std::filesystem::path path);
        void Read(std::filesystem::path path, bool requireDemoMatch = false);

        // the keyframes of this demo, for the next session
        void WriteRecent();
        void ReadRecent();
        // A demo load was asked (the main thread, as the demo command runs): its keys are read once it plays, by
        // ReadRecentWhenLoaded once a frame, which is true when it read them (RecentKeysLogic). Nothing is
        // written while a load waits for its read.
        void AskRecentRead();
        bool ReadRecentWhenLoaded();
        // Saves them for the next session half a second after the last change, so a drag doesn't write the file every
        // frame: WriteRecentIfChanged writes them once that long has passed, or at once with now
        void WriteRecentSoon();
        void WriteRecentIfChanged(bool now = false);
        // a change waits for its save (in a demo), for the harness
        bool IsWriteDue();
    }  // namespace KeyframeSerializer
}  // namespace IWXMVM::Components