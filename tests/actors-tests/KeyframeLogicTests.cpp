#include "StdInclude.hpp"
#include "Test.hpp"

#include "Components/KeyframeLogic.hpp"
#include "Components/UndoHistory.hpp"

using namespace IWXMVM;
using Types::KeyInterpolation;

namespace
{
    const Types::KeyframeableProperty FLOAT_PROPERTY(Types::KeyframeablePropertyType::FilmtweakBrightness, "Brightness",
                                                     Types::KeyframeValueType::FloatingPoint, -1, 1);
    const Types::KeyframeableProperty VECTOR_PROPERTY(Types::KeyframeablePropertyType::SunLightColor, "Sun",
                                                      Types::KeyframeValueType::Vector3, 0, 1);
    const Types::KeyframeableProperty CAMPATH_PROPERTY(Types::KeyframeablePropertyType::CampathCamera, "Campath",
                                                       Types::KeyframeValueType::CameraData, -50, 50);

    std::vector<Types::Keyframe> FloatKeys(std::vector<std::pair<std::uint32_t, float>> points,
                                           KeyInterpolation interpolation)
    {
        std::vector<Types::Keyframe> keys;
        for (const auto& [tick, value] : points)
        {
            keys.emplace_back(FLOAT_PROPERTY, tick, value);
            keys.back().interpolation = interpolation;
        }
        return keys;
    }

    Types::Keyframe Node(std::uint32_t tick, glm::vec3 position, glm::vec3 rotation, float fov,
                         KeyInterpolation interpolation)
    {
        Types::Keyframe node(CAMPATH_PROPERTY, tick, Types::CameraData{position, rotation, fov});
        node.interpolation = interpolation;
        return node;
    }

    float At(const std::vector<Types::Keyframe>& keys, double tick, std::uint32_t index = 0)
    {
        return KeyframeLogic::Evaluate(keys, index, false, tick);
    }

    // KeyframeManager's interpolation before keys had their own, as it was (the reference "Auto" has to match)
    namespace Old
    {
        float InterpolateCubicSpline(const std::vector<Types::Keyframe>& keyframes, uint32_t valueIndex, float tick)
        {
            const size_t n = keyframes.size();
            constexpr int32_t MAX_NODES = 256;
            float ticks[MAX_NODES];
            float values[MAX_NODES];
            for (size_t i = 0; i < n; i++)
            {
                ticks[i] = static_cast<float>(keyframes[i].tick);
                values[i] = keyframes[i].value.GetByIndex(valueIndex);
            }

            float y2[MAX_NODES];  // second derivatives
            float u[MAX_NODES];

            y2[0] = -0.5f;
            u[0] = (3.0f / (ticks[1] - ticks[0])) * ((values[1] - values[0]) / (ticks[1] - ticks[0]));

            for (size_t i = 1; i <= n - 2; i++)
            {
                const auto prevTick = ticks[i - 1];
                const auto prevValue = values[i - 1];
                const auto currTick = ticks[i];
                const auto currValue = values[i];
                const auto nextTick = ticks[i + 1];
                const auto nextValue = values[i + 1];

                auto sig = (currTick - prevTick) / (nextTick - prevTick);
                auto p = sig * y2[i - 1] + 2.0f;
                y2[i] = (sig - 1.0f) / p;
                u[i] =
                    (nextValue - currValue) / (nextTick - currTick) - (currValue - prevValue) / (currTick - prevTick);
                u[i] = (6.0f * u[i] / (nextTick - prevTick) - sig * u[i - 1]) / p;
            }

            auto qn = 0.5f;
            auto un = (3.0f / (ticks[n - 1] - ticks[n - 2])) *
                      (0.0f - (values[n - 1] - values[n - 2]) / (ticks[n - 1] - ticks[n - 2]));

            y2[n - 1] = (un - qn * u[n - 2]) / (qn * y2[n - 2] + 1.0f);

            for (int k = static_cast<int>(n) - 2; k >= 0; k--)
                y2[k] = y2[k] * y2[k + 1] + u[k];

            int klo = 0;
            int khi = static_cast<int>(n) - 1;
            while (khi - klo > 1)
            {
                int k = (khi + klo) >> 1;
                if (ticks[k] > tick)
                    khi = k;
                else
                    klo = k;
            }
            auto h = ticks[khi] - ticks[klo];
            auto a = (ticks[khi] - tick) / h;
            auto b = (tick - ticks[klo]) / h;
            return a * values[klo] + b * values[khi] +
                   ((a * a * a - a) * y2[klo] + (b * b * b - b) * y2[khi]) * (h * h) / 6.0f;
        }

        Types::KeyframeValue LinearlyInterpolate(Types::KeyframeValueType valueType,
                                                 const std::vector<Types::Keyframe>& keyframes, const float tick)
        {
            if (keyframes.size() == 1)
                return keyframes.front().value;

            std::int32_t p0Idx = 0, p1Idx = 1;
            for (std::size_t i = 0; i < keyframes.size() - 1; i++)
            {
                if (tick >= keyframes[i].tick && tick <= keyframes[i + 1].tick)
                {
                    p0Idx = static_cast<std::int32_t>(i);
                    p1Idx = static_cast<std::int32_t>(i + 1);
                    break;
                }
            }

            const auto& p0 = keyframes[p0Idx];
            const auto& p1 = keyframes[p1Idx];

            const float t = static_cast<float>(tick - p0.tick) / static_cast<float>(p1.tick - p0.tick);
            switch (valueType)
            {
                case Types::KeyframeValueType::FloatingPoint:
                    return Types::KeyframeValue((1.0f - t) * p0.value.floatingPoint + t * p1.value.floatingPoint);
                case Types::KeyframeValueType::Vector3:
                    return Types::KeyframeValue((1.0f - t) * p0.value.vector3 + t * p1.value.vector3);
                case Types::KeyframeValueType::CameraData:
                    return Types::KeyframeValue(
                        Types::CameraData((1.0f - t) * p0.value.cameraData.position + t * p1.value.cameraData.position,
                                          (1.0f - t) * p0.value.cameraData.rotation + t * p1.value.cameraData.rotation,
                                          (1.0f - t) * p0.value.cameraData.fov + t * p1.value.cameraData.fov));
                default:
                    return Types::KeyframeValue(0.0f);
            }
        }

        Types::KeyframeValue Interpolate(Types::KeyframeValueType valueType,
                                         const std::vector<Types::Keyframe>& keyframes, const float tick)
        {
            if (tick < keyframes.front().tick)
                return keyframes.front().value;
            if (tick > keyframes.back().tick)
                return keyframes.back().value;
            if (keyframes.size() < 4)
                return LinearlyInterpolate(valueType, keyframes, tick);
            const auto value = [&](std::uint32_t index) { return InterpolateCubicSpline(keyframes, index, tick); };
            switch (valueType)
            {
                case Types::KeyframeValueType::FloatingPoint:
                    return value(0);
                case Types::KeyframeValueType::Vector3:
                    return glm::vec3(value(0), value(1), value(2));
                default:
                    return Types::KeyframeValue(Types::CameraData(glm::vec3(value(0), value(1), value(2)),
                                                                  glm::vec3(value(3), value(4), value(5)), value(6)));
            }
        }
    }  // namespace Old

    // every value of the type, exactly the same
    bool SameValue(Types::KeyframeValueType type, const Types::KeyframeValue& a, const Types::KeyframeValue& b)
    {
        const auto count = type == Types::KeyframeValueType::FloatingPoint ? 1
                           : type == Types::KeyframeValueType::Vector3     ? 3
                                                                           : 7;
        for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(count); ++i)
        {
            if (a.GetByIndex(i) != b.GetByIndex(i))
                return false;
        }
        return true;
    }

    // whether keys of older files come out exactly as the old code had them, at every tick and some between
    bool MatchesOld(Types::KeyframeValueType type, const std::vector<Types::Keyframe>& keys)
    {
        for (float tick = static_cast<float>(keys.front().tick) - 100; tick <= keys.back().tick + 100; tick += 7.5f)
        {
            if (!SameValue(type, Old::Interpolate(type, keys, tick), KeyframeLogic::Evaluate(type, keys, tick)))
                return false;
        }
        return true;
    }

    // A campath from a film: five nodes over 1.71 s and 280 units, which turns about 36
    // degrees at its fourth node; its X, Y, Z, pitch and yaw
    constexpr std::array<std::pair<std::uint32_t, std::array<float, 5>>, 5> FILM_NODES = {{
        {90391, {883.5695f, 2782.9016f, 37.5235f, -15.6f, -172.7f}},
        {90793, {819.1826f, 2740.2185f, 36.1692f, -15.6f, -171.8032f}},
        {91264, {748.2736f, 2724.5781f, 35.9981f, -12.0290f, -176.0921f}},
        {91655, {688.8417f, 2712.8586f, 37.3388f, -6.7787f, -196.5020f}},
        {92102, {626.7946f, 2741.2300f, 35.5504f, -1.5283f, -206.1153f}},
    }};

    std::vector<Types::Keyframe> FilmNodes(KeyInterpolation interpolation, std::size_t count = FILM_NODES.size())
    {
        std::vector<Types::Keyframe> nodes;
        for (std::size_t i = 0; i < count; ++i)
        {
            const auto& [tick, v] = FILM_NODES[i];
            nodes.push_back(Node(tick, {v[0], v[1], v[2]}, {v[3], v[4], 0.2229f}, 66.55f, interpolation));
        }
        return nodes;
    }

    // the same places as keys of a Look property's (a Vector3), whose curves are as they were
    std::vector<Types::Keyframe> FilmPlaces(KeyInterpolation interpolation)
    {
        std::vector<Types::Keyframe> keys;
        for (const auto& [tick, v] : FILM_NODES)
        {
            keys.emplace_back(VECTOR_PROPERTY, tick, glm::vec3(v[0], v[1], v[2]));
            keys.back().interpolation = interpolation;
        }
        return keys;
    }

    // how a camera on the keys moves over frames this many ms apart, from the first key to the last
    KeyframeLogic::PathProfile ProfileOf(const std::vector<Types::Keyframe>& keys, double frameMs = 20.0)
    {
        const auto type = keys.front().property.get().valueType;
        std::vector<KeyframeLogic::PathSample> samples;
        const auto add = [&](double tick) {
            const auto value = KeyframeLogic::Evaluate(type, keys, tick);
            samples.push_back(
                {tick, type == Types::KeyframeValueType::CameraData ? value.cameraData.position : value.vector3});
        };
        for (double tick = keys.front().tick; tick < keys.back().tick; tick += frameMs)
            add(tick);
        add(keys.back().tick);
        return KeyframeLogic::MakeProfile(samples);
    }

    // how much a camera's acceleration along its way jumps at a tick (u/s2), from its speed over 5 ms on either side
    double AccelerationJump(const std::vector<Types::Keyframe>& nodes, double tick)
    {
        const auto at = [&](double t) {
            return KeyframeLogic::Evaluate(Types::KeyframeValueType::CameraData, nodes, t).cameraData.position;
        };
        constexpr double STEP = 5.0;
        const auto speed = [&](double from) { return glm::distance(at(from), at(from + STEP)) / (STEP / 1000.0); };
        const auto before = (speed(tick - STEP) - speed(tick - 2 * STEP)) / (STEP / 1000.0);
        const auto after = (speed(tick + STEP) - speed(tick)) / (STEP / 1000.0);
        return std::abs(after - before);
    }
}  // namespace

TEST("keys of older files (Auto) go as they did: linear under four keys")
{
    CHECK(MatchesOld(Types::KeyframeValueType::FloatingPoint, FloatKeys({{1000, 0.5f}}, KeyInterpolation::Auto)));
    CHECK(MatchesOld(Types::KeyframeValueType::FloatingPoint,
                     FloatKeys({{1000, 0.5f}, {2500, -0.25f}, {2600, 1.0f}}, KeyInterpolation::Auto)));

    std::vector<Types::Keyframe> sun;
    for (const auto& [tick, color] : {std::pair(20000u, glm::vec3(1, 0.5f, 0.2f)), {23000u, glm::vec3(0.3f, 0.9f, 0)}})
    {
        sun.emplace_back(VECTOR_PROPERTY, tick, color);
        sun.back().interpolation = KeyInterpolation::Auto;
    }
    CHECK(MatchesOld(Types::KeyframeValueType::Vector3, sun));
}

TEST("keys of older files (Auto) go as they did: the cubic spline through all of them from four keys")
{
    CHECK(MatchesOld(
        Types::KeyframeValueType::FloatingPoint,
        FloatKeys({{1000, 0.5f}, {2500, -0.25f}, {2600, 1.0f}, {4000, 0.0f}, {9000, 0.75f}}, KeyInterpolation::Auto)));
    CHECK(MatchesOld(Types::KeyframeValueType::Vector3, FilmPlaces(KeyInterpolation::Auto)));
}

TEST("the campath's Auto goes through its first and last node: only the eases slow its ends (#147)")
{
    // IWXMVM's spline, at rest at both ends, on the film's nodes: 0 u/s at the ends, 259 at most, and the brake into
    // the last node starting at the fourth (the Look's Auto still has it)
    const auto old = ProfileOf(FilmPlaces(KeyInterpolation::Auto));
    CHECK(old.first < 30.0 && old.last < 30.0);
    CHECK(old.most > 250.0);

    const auto nodes = FilmNodes(KeyInterpolation::Auto);
    const auto profile = ProfileOf(nodes);
    CHECK(profile.first > 195.0 && profile.first < 215.0);
    CHECK(profile.last > 150.0 && profile.last < 175.0);
    CHECK(profile.least > 140.0);
    CHECK(profile.most < 215.0);
    // one curve: no step of acceleration at a node, no corner
    CHECK(profile.accelerationStep < 30.0);
    CHECK(profile.turn < 5.0);
    // through every node
    const auto at = [&](const std::vector<Types::Keyframe>& keys, double tick) {
        return KeyframeLogic::Evaluate(Types::KeyframeValueType::CameraData, keys, tick).cameraData.position;
    };
    for (const auto& node : nodes)
        CHECK_NEAR(at(nodes, node.tick).y, node.value.cameraData.position.y, 1e-3);

    // an ease of 1 leaves and arrives at rest; 30% slows it by 30%
    auto eased = nodes;
    eased.front().easeOut = 1.0f;
    eased.back().easeIn = 1.0f;
    // (frames 2 ms apart: over 20 ms it reaches 20 u/s already)
    const auto atRest = ProfileOf(eased, 2.0);
    CHECK(atRest.first < 5.0 && atRest.last < 5.0);
    eased.front().easeOut = 0.3f;
    eased.back().easeIn = 0.0f;
    const auto slowed = ProfileOf(eased);
    CHECK(slowed.first > 0.65 * profile.first && slowed.first < 0.8 * profile.first);
    CHECK_NEAR(slowed.last, profile.last, 0.5);
    // the stretches between are the same
    CHECK_NEAR(KeyframeLogic::Evaluate(Types::KeyframeValueType::CameraData, eased, 91400).cameraData.position.x,
               KeyframeLogic::Evaluate(Types::KeyframeValueType::CameraData, nodes, 91400).cameraData.position.x,
               1e-4);

    // Auto Ease's 30% at both ends leaves the second node and the one before the last as the curve has them (the
    // eases' old shape bent the far end of the stretch: a jump of acceleration there)
    eased.front().easeOut = 0.3f;
    eased.back().easeIn = 0.3f;
    CHECK(AccelerationJump(nodes, 90793) < 30.0);
    CHECK(AccelerationJump(eased, 90793) < 30.0);
    CHECK(AccelerationJump(eased, 91655) < 30.0);
    KeyframeLogic::SetPathCurves(false);
    const auto oldJump = AccelerationJump(eased, 90793);
    KeyframeLogic::SetPathCurves(true);
    CHECK(oldJump > 150.0);
}

TEST("the campath's Auto curves from three nodes, and two make a straight line (#147)")
{
    // the film's first three nodes turn 21 degrees at the second: a corner as straight lines, a curve now
    const auto old = ProfileOf(FilmPlaces(KeyInterpolation::Linear));
    CHECK(old.turn > 20.0);
    const auto three = FilmNodes(KeyInterpolation::Auto, 3);
    const auto profile = ProfileOf(three);
    CHECK(profile.turn < 5.0);
    CHECK(profile.least > 100.0);
    const auto middle = KeyframeLogic::Evaluate(Types::KeyframeValueType::CameraData, three, 91000).cameraData;
    const auto straight = glm::mix(three[1].value.cameraData.position, three[2].value.cameraData.position,
                                   (91000.0f - 90793.0f) / (91264.0f - 90793.0f));
    CHECK(glm::distance(middle.position, straight) > 0.5f);

    const auto two = FilmNodes(KeyInterpolation::Auto, 2);
    const auto halfway = KeyframeLogic::Evaluate(Types::KeyframeValueType::CameraData, two, 90592).cameraData;
    const auto between = glm::mix(two[0].value.cameraData.position, two[1].value.cameraData.position, 0.5f);
    CHECK(glm::distance(halfway.position, between) < 0.01f);
    const auto line = ProfileOf(two);
    CHECK_NEAR(line.least, line.most, 0.5);
}

TEST("the campath's Smooth stops no axis at a node, and goes through its ends (#147)")
{
    // the Look's Smooth on the same places: each axis stops where it turns (Y at the fourth node), and at the ends
    const auto look = ProfileOf(FilmPlaces(KeyInterpolation::Smooth));
    CHECK(look.first < 30.0 && look.last < 30.0);
    CHECK(look.accelerationStep > 500.0);
    const auto places = FilmPlaces(KeyInterpolation::Smooth);
    const auto ys = KeyframeLogic::GetValues(places, 1, false);
    CHECK(KeyframeLogic::GetSmoothSlope(places, ys, 3) == 0.0);

    const auto nodes = FilmNodes(KeyInterpolation::Smooth);
    CHECK(KeyframeLogic::GetPathSlope(nodes, KeyframeLogic::GetValues(nodes, 1, false), 3) != 0.0);
    const auto profile = ProfileOf(nodes);
    CHECK(profile.first > 190.0 && profile.last > 150.0);
    CHECK(profile.least > 140.0);
    CHECK(profile.most < 215.0);
    CHECK(profile.accelerationStep < 60.0);
    CHECK(profile.turn < 5.0);
    for (const auto& node : nodes)
    {
        const auto value = KeyframeLogic::Evaluate(Types::KeyframeValueType::CameraData, nodes, node.tick);
        CHECK_NEAR(value.cameraData.position.x, node.value.cameraData.position.x, 1e-3);
    }
    // the same on both sides of a node
    for (const std::uint32_t tick : {90793u, 91655u})
    {
        const auto at = [&](double t) {
            return KeyframeLogic::Evaluate(Types::KeyframeValueType::CameraData, nodes, t).cameraData.position.y;
        };
        CHECK_NEAR((at(tick) - at(tick - 0.5)) / 0.5, (at(tick + 0.5) - at(tick)) / 0.5, 0.002);
    }
    // two nodes: a straight line, as Auto and Linear
    const auto two = FilmNodes(KeyInterpolation::Smooth, 2);
    const auto line = ProfileOf(two);
    CHECK_NEAR(line.least, line.most, 0.5);
}

TEST("a profile of frames: the speeds, the biggest change of acceleration and the biggest turn")
{
    // a straight line at 100 u/s, a frame each 20 ms, one frame given twice
    std::vector<KeyframeLogic::PathSample> line;
    for (int i = 0; i <= 50; ++i)
        line.push_back({i * 20.0, glm::vec3(i * 2.0f, 0, 0)});
    line.insert(line.begin() + 10, line[10]);
    const auto straight = KeyframeLogic::MakeProfile(line);
    CHECK(straight.frames == 51);
    CHECK_NEAR(straight.least, 100.0, 1e-3);
    CHECK_NEAR(straight.most, 100.0, 1e-3);
    CHECK(straight.accelerationStep < 1e-3);
    CHECK(straight.turn < 1e-3);

    // a corner, then twice the speed from one frame to the next
    const std::vector<KeyframeLogic::PathSample> corner = {
        {0, {0, 0, 0}}, {20, {2, 0, 0}}, {40, {4, 0, 0}}, {60, {4, 2, 0}}, {80, {4, 6, 0}}, {100, {4, 10, 0}}};
    const auto turned = KeyframeLogic::MakeProfile(corner);
    CHECK_NEAR(turned.turn, 90.0, 1e-3);
    CHECK_NEAR(turned.turnTick, 40.0, 1e-3);
    CHECK_NEAR(turned.least, 100.0, 1e-3);
    CHECK_NEAR(turned.most, 200.0, 1e-3);
    // 0, then 5,000 u/s2 for one frame, then 0 again
    CHECK_NEAR(turned.accelerationStep, 5000.0, 1e-3);
}

TEST("the interpolation all the keys share, and none when they differ")
{
    CHECK(!KeyframeLogic::GetSharedInterpolation({}).has_value());
    auto nodes = FilmNodes(KeyInterpolation::Auto);
    CHECK(KeyframeLogic::GetSharedInterpolation(nodes) == KeyInterpolation::Auto);
    nodes[2].interpolation = KeyInterpolation::Smooth;
    CHECK(!KeyframeLogic::GetSharedInterpolation(nodes).has_value());
}

TEST("Linear goes straight from key to key, Hold stays until the next key")
{
    const auto linear = FloatKeys({{0, 0.0f}, {1000, 10.0f}, {3000, -10.0f}}, KeyInterpolation::Linear);
    CHECK_NEAR(At(linear, 500), 5.0f, 1e-5);
    CHECK_NEAR(At(linear, 2000), 0.0f, 1e-5);
    CHECK_NEAR(At(linear, 2500), -5.0f, 1e-5);
    // before the first key and after the last
    CHECK_NEAR(At(linear, -100), 0.0f, 1e-6);
    CHECK_NEAR(At(linear, 5000), -10.0f, 1e-6);

    const auto hold = FloatKeys({{0, 0.0f}, {1000, 10.0f}, {3000, -10.0f}}, KeyInterpolation::Hold);
    CHECK_NEAR(At(hold, 999.9), 0.0f, 1e-6);
    CHECK_NEAR(At(hold, 1000), 10.0f, 1e-6);
    CHECK_NEAR(At(hold, 2999), 10.0f, 1e-6);
    CHECK_NEAR(At(hold, 3000), -10.0f, 1e-6);
}

TEST("Smooth goes through every key and rests at the first and the last")
{
    const auto keys = FloatKeys({{0, 0.0f}, {1000, 4.0f}, {2500, 5.0f}, {4000, 9.0f}}, KeyInterpolation::Smooth);
    for (const auto& key : keys)
        CHECK_NEAR(At(keys, key.tick), key.value.floatingPoint, 1e-5);
    // flat at the ends
    CHECK_NEAR((At(keys, 1) - At(keys, 0)), 0.0f, 1e-4);
    CHECK_NEAR((At(keys, 4000) - At(keys, 3999)), 0.0f, 1e-4);
    // two keys: an S from one to the other
    const auto two = FloatKeys({{0, 0.0f}, {1000, 1.0f}}, KeyInterpolation::Smooth);
    CHECK_NEAR(At(two, 500), 0.5f, 1e-6);
    CHECK_NEAR(At(two, 250), 0.15625f, 1e-6);
}

TEST("Smooth doesn't overshoot: it stays between the keys around it, and at a key where the values turn")
{
    // rising, with a jump in the middle (a spline through them would swing past both sides of it)
    const auto rising =
        FloatKeys({{0, 0.0f}, {1000, 0.1f}, {1100, 5.0f}, {2000, 5.1f}, {3000, 5.2f}}, KeyInterpolation::Smooth);
    float last = -1.0f;
    for (double tick = 0; tick <= 3000; tick += 5)
    {
        const auto value = At(rising, tick);
        CHECK(value >= last - 1e-5f);
        last = value;
    }
    CHECK(At(rising, 1500) <= 5.1f + 1e-5f);
    CHECK(At(rising, 500) <= 0.1f + 1e-5f);

    // a peak: nothing above it
    const auto peak = FloatKeys({{0, 0.0f}, {1000, 3.0f}, {1200, 2.9f}, {3000, 0.0f}}, KeyInterpolation::Smooth);
    for (double tick = 0; tick <= 3000; tick += 5)
        CHECK(At(peak, tick) <= 3.0f + 1e-5f);
}

TEST("Smooth's slope is the same on both sides of a key")
{
    const auto keys = FloatKeys({{0, 0.0f}, {1000, 4.0f}, {2500, 5.0f}, {4000, 9.0f}}, KeyInterpolation::Smooth);
    for (const std::uint32_t tick : {1000u, 2500u})
    {
        const auto before = (At(keys, tick) - At(keys, tick - 0.5)) / 0.5;
        const auto after = (At(keys, tick + 0.5) - At(keys, tick)) / 0.5;
        CHECK(before > 0.0);
        CHECK_NEAR(before, after, std::abs(after) * 0.01 + 1e-5);
    }
}

TEST("each key's interpolation decides the way to the next key")
{
    auto keys = FloatKeys({{0, 0.0f}, {1000, 10.0f}, {2000, 0.0f}, {3000, 10.0f}}, KeyInterpolation::Smooth);
    keys[0].interpolation = KeyInterpolation::Linear;
    keys[1].interpolation = KeyInterpolation::Hold;
    CHECK_NEAR(At(keys, 500), 5.0f, 1e-5);
    CHECK_NEAR(At(keys, 1500), 10.0f, 1e-6);
    // the last way is Smooth: an S from 0 to 10, which rests at the last key
    CHECK_NEAR(At(keys, 2500), 5.0f, 1e-4);
    CHECK(At(keys, 2250) < 2.5f);
}

TEST("eases slow a way down at its ends, and never take it back")
{
    CHECK_NEAR(KeyframeLogic::Ease(0.3, 0.0f, 0.0f), 0.3, 1e-12);
    CHECK_NEAR(KeyframeLogic::Ease(0.0, 1.0f, 1.0f), 0.0, 1e-12);
    CHECK_NEAR(KeyframeLogic::Ease(1.0, 1.0f, 1.0f), 1.0, 1e-12);
    // at rest at the eased end
    CHECK(KeyframeLogic::Ease(0.01, 1.0f, 0.0f) < 0.001);
    CHECK_NEAR(KeyframeLogic::Ease(0.99, 1.0f, 0.0f), 0.99, 0.005);
    CHECK(1.0 - KeyframeLogic::Ease(0.99, 0.0f, 1.0f) < 0.001);
    for (const auto [easeOut, easeIn] : {std::pair(1.0f, 1.0f), {0.5f, 0.0f}, {0.0f, 1.0f}, {0.3f, 0.8f}})
    {
        double last = 0.0;
        for (double t = 0.0; t <= 1.0; t += 0.01)
        {
            const auto eased = KeyframeLogic::Ease(t, easeOut, easeIn);
            CHECK(eased >= last - 1e-12);
            last = eased;
            CHECK_NEAR(KeyframeLogic::Uneased(eased, easeOut, easeIn), t, 1e-6);
        }
    }

    // on keys: a key's ease out and the next key's ease in
    auto keys = FloatKeys({{0, 0.0f}, {1000, 10.0f}}, KeyInterpolation::Linear);
    keys[0].easeOut = 1.0f;
    CHECK(At(keys, 100) < 0.5f);
    CHECK_NEAR(At(keys, 1000), 10.0f, 1e-6);
    keys[0].easeOut = 0.0f;
    keys[1].easeIn = 1.0f;
    CHECK(At(keys, 900) > 9.5f);
}

TEST("angles are unwrapped against the key before: the camera turns the short way round")
{
    std::vector<float> yaws = {170.0f, -170.0f, 150.0f, 900.0f};
    KeyframeLogic::Unwrap(yaws);
    CHECK_NEAR(yaws[1], 190.0f, 1e-4);
    CHECK_NEAR(yaws[2], 150.0f, 1e-4);
    CHECK_NEAR(yaws[3], 180.0f, 1e-4);

    // one node placed from the player's view at 170, the next in the free camera at -170: through 180, not 0
    const std::vector nodes = {Node(0, {0, 0, 0}, {0, 170, 0}, 80, KeyInterpolation::Linear),
                               Node(1000, {100, 0, 0}, {0, -170, 0}, 80, KeyInterpolation::Linear)};
    const auto middle = KeyframeLogic::Evaluate(Types::KeyframeValueType::CameraData, nodes, 500.0);
    CHECK_NEAR(middle.cameraData.rotation.y, 180.0f, 1e-3);
    CHECK_NEAR(middle.cameraData.position.x, 50.0f, 1e-3);
    // the free camera's yaw two turns on, and roll too
    const std::vector spun = {Node(0, {0, 0, 0}, {0, 10, -175}, 80, KeyInterpolation::Smooth),
                              Node(1000, {0, 0, 0}, {0, 730 + 20, 175}, 80, KeyInterpolation::Smooth)};
    const auto halfway = KeyframeLogic::Evaluate(Types::KeyframeValueType::CameraData, spun, 500.0);
    CHECK_NEAR(halfway.cameraData.rotation.y, 20.0f, 1e-3);
    CHECK_NEAR(halfway.cameraData.rotation.z, -180.0f, 1e-3);
    // older files' campaths too
    auto old = nodes;
    for (auto& node : old)
        node.interpolation = KeyInterpolation::Auto;
    CHECK_NEAR(KeyframeLogic::Evaluate(Types::KeyframeValueType::CameraData, old, 500.0).cameraData.rotation.y, 180.0f,
               1e-3);
    // other values aren't angles
    CHECK(!KeyframeLogic::IsAngle(Types::KeyframeValueType::Vector3, 1));
    CHECK(KeyframeLogic::IsAngle(Types::KeyframeValueType::CameraData, 4));
    CHECK(!KeyframeLogic::IsAngle(Types::KeyframeValueType::CameraData, 6));
}

TEST("a key added takes the interpolation of the key before it, so older campaths keep going as they did")
{
    using Types::KeyframeablePropertyType;
    auto keys = FloatKeys({{1000, 0.0f}, {2000, 1.0f}}, KeyInterpolation::Auto);
    keys[1].interpolation = KeyInterpolation::Linear;
    const auto type = KeyframeablePropertyType::FilmtweakBrightness;
    CHECK(KeyframeLogic::GetNewKeyInterpolation(keys, 1500, type) == KeyInterpolation::Auto);
    CHECK(KeyframeLogic::GetNewKeyInterpolation(keys, 2500, type) == KeyInterpolation::Linear);
    CHECK(KeyframeLogic::GetNewKeyInterpolation(keys, 500, type) == KeyInterpolation::Auto);
    // the first key: Auto for the campath (as IWXMVM moved the camera), Smooth for the rest
    CHECK(KeyframeLogic::GetNewKeyInterpolation({}, 500, KeyframeablePropertyType::CampathCamera) ==
          KeyInterpolation::Auto);
    CHECK(KeyframeLogic::GetNewKeyInterpolation({}, 500, type) == KeyInterpolation::Smooth);
    CHECK(KeyframeLogic::GetNewKeyInterpolation({}, 500, KeyframeablePropertyType::SunLightDirection) ==
          KeyInterpolation::Smooth);
    // a campath made Smooth stays Smooth as nodes are added to it
    auto smooth = FloatKeys({{1000, 0.0f}, {2000, 1.0f}}, KeyInterpolation::Smooth);
    CHECK(KeyframeLogic::GetNewKeyInterpolation(smooth, 3000, KeyframeablePropertyType::CampathCamera) ==
          KeyInterpolation::Smooth);
}

TEST("a switch's keys hold it on or off until the next key, whatever a key says about interpolation")
{
    using Types::KeyframeablePropertyType;
    for (const auto type : {KeyframeablePropertyType::FilmtweakEnabled, KeyframeablePropertyType::FilmtweakInvert,
                            KeyframeablePropertyType::DepthOfFieldEnabled})
    {
        CHECK(KeyframeLogic::IsStepped(type));
        CHECK(KeyframeLogic::GetDefaultInterpolation(type) == KeyInterpolation::Hold);
        // (even next to a key that has another, as a hand-made file can)
        const auto smooth = FloatKeys({{1000, 0.0f}, {2000, 1.0f}}, KeyInterpolation::Smooth);
        CHECK(KeyframeLogic::GetNewKeyInterpolation(smooth, 1500, type) == KeyInterpolation::Hold);
    }
    CHECK(!KeyframeLogic::IsStepped(KeyframeablePropertyType::FilmtweakBrightness));
    CHECK(!KeyframeLogic::IsStepped(KeyframeablePropertyType::FilmtweakTemperatureRanges));
    CHECK(!KeyframeLogic::IsStepped(KeyframeablePropertyType::CampathCamera));
    CHECK(KeyframeLogic::GetDefaultInterpolation(KeyframeablePropertyType::FilmtweakMidStart) ==
          KeyInterpolation::Smooth);

    // a key as a switch can have it: Hold, no eases, on or off
    auto key = FloatKeys({{1000, 0.7f}}, KeyInterpolation::Smooth).front();
    key.easeIn = 0.5f;
    key.easeOut = 1.0f;
    const auto normalized = KeyframeLogic::Normalize(key, KeyframeablePropertyType::FilmtweakInvert);
    CHECK(normalized.interpolation == KeyInterpolation::Hold);
    CHECK(normalized.easeIn == 0.0f);
    CHECK(normalized.easeOut == 0.0f);
    CHECK(normalized.value.floatingPoint == 1.0f);
    CHECK(normalized.id == key.id);
    CHECK(KeyframeLogic::Normalize(key, KeyframeablePropertyType::FilmtweakBrightness).value.floatingPoint == 0.7f);
    CHECK(KeyframeLogic::GetStepValue(0.49f) == 0.0f);
    CHECK(KeyframeLogic::GetStepValue(0.5f) == 1.0f);

    // off, on at 2000, off again at 3000: no value between
    const auto keys = FloatKeys({{1000, 0.0f}, {2000, 1.0f}, {3000, 0.0f}}, KeyInterpolation::Hold);
    CHECK(At(keys, 1999) == 0.0f);
    CHECK(At(keys, 2000) == 1.0f);
    CHECK(At(keys, 2999) == 1.0f);
    CHECK(At(keys, 3000) == 0.0f);
    CHECK(At(keys, 500) == 0.0f);
}

TEST("Auto Ease eases a new campath's ends, and the ends as nodes are added and deleted")
{
    // nodes made with it on follow it on both sides (KeyframeManager::AddKeyframe)
    const auto made = [](std::uint32_t tick, float x) {
        auto node = Node(tick, glm::vec3(x, 0.0f, 0.0f), glm::vec3(0.0f), 90.0f, KeyInterpolation::Auto);
        node.autoEaseIn = node.autoEaseOut = true;
        return node;
    };
    std::vector<Types::Keyframe> nodes{made(1000, 0.0f), made(3000, 600.0f)};
    CHECK(KeyframeLogic::ApplyAutoEase(nodes, 0.3f, 0.4f));
    CHECK(nodes[0].easeOut == 0.3f && nodes[0].easeIn == 0.0f && nodes[1].easeIn == 0.4f && nodes[1].easeOut == 0.0f);
    CHECK(!KeyframeLogic::ApplyAutoEase(nodes, 0.3f, 0.4f));
    // the camera leaves the first node at 70% of its speed and arrives at the last at 60% (the stretch's slope)
    const auto slope = [&](double fraction) {
        return (KeyframeLogic::Ease(fraction + 1e-4, nodes[0].easeOut, nodes[1].easeIn) -
                KeyframeLogic::Ease(fraction, nodes[0].easeOut, nodes[1].easeIn)) /
               1e-4;
    };
    CHECK_NEAR(slope(0.0), 0.7, 1e-3);
    CHECK_NEAR(slope(1.0 - 1e-4), 0.6, 1e-3);

    // G1: a node added after the last takes the ease end, the old last lets go of it; the start stays
    nodes.push_back(made(5000, 1200.0f));
    KeyframeLogic::ApplyAutoEase(nodes, 0.3f, 0.4f);
    CHECK(nodes[1].easeIn == 0.0f && nodes[2].easeIn == 0.4f && nodes[0].easeOut == 0.3f);
    // and one before the first takes the start
    nodes.insert(nodes.begin(), made(0, -600.0f));
    KeyframeLogic::ApplyAutoEase(nodes, 0.3f, 0.4f);
    CHECK(nodes[0].easeOut == 0.3f && nodes[1].easeOut == 0.0f);
    // deleting the last gives it back to the one before
    nodes.pop_back();
    KeyframeLogic::ApplyAutoEase(nodes, 0.3f, 0.4f);
    CHECK(nodes.back().easeIn == 0.4f);
}

TEST("Auto Ease leaves an ease changed by hand alone, through adds, deletes and its own changes")
{
    auto first = Node(1000, glm::vec3(0.0f), glm::vec3(0.0f), 90.0f, KeyInterpolation::Auto);
    first.autoEaseIn = first.autoEaseOut = true;
    auto last = Node(3000, glm::vec3(600.0f, 0.0f, 0.0f), glm::vec3(0.0f), 90.0f, KeyInterpolation::Auto);
    last.autoEaseIn = last.autoEaseOut = true;
    std::vector<Types::Keyframe> nodes{first, last};
    KeyframeLogic::ApplyAutoEase(nodes, 0.3f, 0.3f);

    // the first node's ease out set by hand to 80%: its own from then on, its ease in still follows
    auto changed = nodes[0];
    changed.easeOut = 0.8f;
    KeyframeLogic::KeepChangedEases(nodes[0], changed);
    CHECK(!changed.autoEaseOut && changed.autoEaseIn);
    nodes[0] = changed;
    // the sliders (L1) move the end that still follows, not the one changed by hand
    KeyframeLogic::ApplyAutoEase(nodes, 0.5f, 0.6f);
    CHECK(nodes[0].easeOut == 0.8f && nodes[1].easeIn == 0.6f);
    // a node added before it doesn't take its ease, nor does it lose it
    auto before = Node(0, glm::vec3(-600.0f, 0.0f, 0.0f), glm::vec3(0.0f), 90.0f, KeyInterpolation::Auto);
    before.autoEaseIn = before.autoEaseOut = true;
    nodes.insert(nodes.begin(), before);
    KeyframeLogic::ApplyAutoEase(nodes, 0.5f, 0.6f);
    CHECK(nodes[0].easeOut == 0.5f && nodes[1].easeOut == 0.8f);
    nodes.erase(nodes.begin());
    KeyframeLogic::ApplyAutoEase(nodes, 0.5f, 0.6f);
    CHECK(nodes[0].easeOut == 0.8f);
    // the Auto button gives it back (the flag again, the value as it was)
    nodes[0].autoEaseOut = true;
    KeyframeLogic::ApplyAutoEase(nodes, 0.5f, 0.6f);
    CHECK(nodes[0].easeOut == 0.5f);
    // a change that leaves an ease as it was (another interpolation) keeps it following
    auto relinked = nodes[1];
    relinked.interpolation = KeyInterpolation::Linear;
    KeyframeLogic::KeepChangedEases(nodes[1], relinked);
    CHECK(relinked.autoEaseIn);

    // nodes made with it off follow nothing: no ease added, none taken (the same for older files' nodes)
    auto own = Node(1000, glm::vec3(0.0f), glm::vec3(0.0f), 90.0f, KeyInterpolation::Auto);
    own.easeIn = 0.2f;
    std::vector<Types::Keyframe> older{
        own, Node(3000, glm::vec3(600.0f, 0.0f, 0.0f), glm::vec3(0.0f), 90.0f, KeyInterpolation::Auto)};
    CHECK(!KeyframeLogic::ApplyAutoEase(older, 0.5f, 0.6f));
    CHECK(older[0].easeIn == 0.2f && older[0].easeOut == 0.0f && older[1].easeIn == 0.0f);
}

TEST("at constant speed the camera goes along the path at one speed from the first node to the last")
{
    // a straight path, its second node early: by the nodes' times the camera would rush, then crawl
    const std::vector nodes = {Node(0, {0, 0, 0}, {0, 0, 0}, 80, KeyInterpolation::Linear),
                               Node(1000, {900, 0, 0}, {0, 90, 0}, 80, KeyInterpolation::Linear),
                               Node(10000, {1000, 0, 0}, {0, 100, 0}, 80, KeyInterpolation::Linear)};
    const auto retiming = KeyframeLogic::MakeRetiming(nodes);
    CHECK_NEAR(retiming.GetLength(), 1000.0, 1e-3);
    for (const double tick : {0.0, 2500.0, 5000.0, 9000.0, 10000.0})
        CHECK_NEAR(KeyframeLogic::EvaluateRetimed(retiming, nodes, tick).cameraData.position.x, tick / 10.0, 0.05);
    // the angles come with the place on the path: the second node's where the camera passes it
    const auto passing = KeyframeLogic::GetPassingTick(retiming, 1000.0);
    CHECK_NEAR(passing, 9000.0, 1.0);
    CHECK_NEAR(KeyframeLogic::EvaluateRetimed(retiming, nodes, passing).cameraData.rotation.y, 90.0f, 0.05);
    // before and after the nodes, as the nodes have it
    CHECK_NEAR(KeyframeLogic::EvaluateRetimed(retiming, nodes, -50).cameraData.position.x, 0.0f, 1e-4);
    CHECK_NEAR(KeyframeLogic::EvaluateRetimed(retiming, nodes, 12000).cameraData.position.x, 1000.0f, 1e-4);

    // a curved path: the speed is the same all along it
    const std::vector curve = {Node(0, {0, 0, 0}, {}, 80, KeyInterpolation::Smooth),
                               Node(300, {500, 500, 0}, {}, 80, KeyInterpolation::Smooth),
                               Node(5000, {1000, 0, 100}, {}, 80, KeyInterpolation::Smooth)};
    const auto curved = KeyframeLogic::MakeRetiming(curve);
    const auto speed = curved.GetLength() / 5000.0;
    for (double tick = 500; tick < 4500; tick += 500)
    {
        const auto a = KeyframeLogic::EvaluateRetimed(curved, curve, tick).cameraData.position;
        const auto b = KeyframeLogic::EvaluateRetimed(curved, curve, tick + 10).cameraData.position;
        CHECK_NEAR(glm::distance(a, b) / 10.0, speed, speed * 0.02);
    }

    // the first node's ease out and the last's ease in still slow its ends
    auto eased = nodes;
    eased.front().easeOut = 1.0f;
    const auto easedRetiming = KeyframeLogic::MakeRetiming(eased);
    CHECK(KeyframeLogic::EvaluateRetimed(easedRetiming, eased, 500).cameraData.position.x < 10.0f);
    CHECK_NEAR(KeyframeLogic::EvaluateRetimed(easedRetiming, eased, 10000).cameraData.position.x, 1000.0f, 1e-3);
}

TEST("Auto Ease on all nodes eases every stretch, new nodes too, and leaves eases changed by hand alone (#18)")
{
    const auto made = [](std::uint32_t tick, float x) {
        auto node = Node(tick, glm::vec3(x, 0.0f, 0.0f), glm::vec3(0.0f), 90.0f, KeyInterpolation::Auto);
        node.autoEaseIn = node.autoEaseOut = true;
        return node;
    };
    const auto all = Types::AutoEaseNodes::AllNodes;
    std::vector<Types::Keyframe> nodes{made(1000, 0.0f), made(3000, 600.0f), made(5000, 1200.0f)};
    CHECK(KeyframeLogic::ApplyAutoEase(nodes, 0.3f, 0.4f, all));
    // out of each node by the start, into each by the end; the first's ease in and the last's ease out lead nowhere
    CHECK(nodes[0].easeIn == 0.0f && nodes[0].easeOut == 0.3f);
    CHECK(nodes[1].easeIn == 0.4f && nodes[1].easeOut == 0.3f);
    CHECK(nodes[2].easeIn == 0.4f && nodes[2].easeOut == 0.0f);
    CHECK(!KeyframeLogic::ApplyAutoEase(nodes, 0.3f, 0.4f, all));

    // a node placed afterwards, after the last and between two: eased like the rest, the old last now easing out
    nodes.push_back(made(7000, 1800.0f));
    nodes.insert(nodes.begin() + 1, made(2000, 300.0f));
    KeyframeLogic::ApplyAutoEase(nodes, 0.3f, 0.4f, all);
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        CHECK(nodes[i].easeIn == (i == 0 ? 0.0f : 0.4f));
        CHECK(nodes[i].easeOut == (i + 1 == nodes.size() ? 0.0f : 0.3f));
    }

    // a middle node's ease in changed by hand: its own through the sliders and the switch back to First and Last
    auto changed = nodes[2];
    changed.easeIn = 0.9f;
    KeyframeLogic::KeepChangedEases(nodes[2], changed);
    nodes[2] = changed;
    KeyframeLogic::ApplyAutoEase(nodes, 0.5f, 0.6f, all);
    CHECK(nodes[2].easeIn == 0.9f && nodes[2].easeOut == 0.5f && nodes[1].easeIn == 0.6f);
    KeyframeLogic::ApplyAutoEase(nodes, 0.5f, 0.6f, Types::AutoEaseNodes::FirstAndLast);
    CHECK(nodes[2].easeIn == 0.9f && nodes[2].easeOut == 0.0f && nodes[1].easeIn == 0.0f);
    CHECK(nodes.front().easeOut == 0.5f && nodes.back().easeIn == 0.6f);

    // nodes that follow nothing (made with it off, older files) get none on all nodes either
    auto own = Node(9000, glm::vec3(2400.0f, 0.0f, 0.0f), glm::vec3(0.0f), 90.0f, KeyInterpolation::Auto);
    nodes.push_back(own);
    KeyframeLogic::ApplyAutoEase(nodes, 0.5f, 0.6f, all);
    CHECK(nodes.back().easeIn == 0.0f && nodes.back().easeOut == 0.0f);
}

namespace
{
    // what the view does each frame: a frame's gizmo changes the node, then says what it held (the node as it was
    // before that frame), and a drag that ended is a step unless it moved nothing (KeyframeManager::EndNodeDrag)
    struct NodeGizmo
    {
        Types::Keyframe& node;
        Components::UndoHistory history;
        KeyframeLogic::KeyDrag drag;
        std::vector<std::string> labels;

        void Frame(bool held, glm::vec3 moved = glm::vec3(0.0f), glm::vec3 turned = glm::vec3(0.0f))
        {
            const auto before = node;
            node.value.cameraData.position += moved;
            node.value.cameraData.rotation += turned;
            const auto ended = drag.Frame(held ? &before : nullptr);
            if (!ended)
                return;
            if (KeyframeLogic::IsSameCamera(ended->before.cameraData, node.value.cameraData))
            {
                node.value = ended->before;
                return;
            }
            const auto was = ended->before;
            const auto now = node.value;
            labels.emplace_back(KeyframeLogic::GetNodeDragVerb(was.cameraData, now.cameraData));
            history.Push(labels.back(), [this, was] { node.value = was; }, [this, now] { node.value = now; });
        }
    };
}  // namespace

TEST("a drag of a node's gizmo is one step, however many frames it moved it, undone and redone whole")
{
    auto node = Node(20000, {100, 200, 30}, {10, 45, 0}, 80, KeyInterpolation::Auto);
    NodeGizmo gizmo{node};

    // grabbed (that frame takes it as it was), moved over five frames, let go
    for (int i = 0; i < 5; ++i)
        gizmo.Frame(true, glm::vec3(10, 0, 0));
    CHECK(!gizmo.history.GetUndoLabel());
    gizmo.Frame(false);
    CHECK(gizmo.history.GetUndoLabel() == "move");
    CHECK_NEAR(node.value.cameraData.position.x, 150.0f, 1e-3f);

    CHECK(gizmo.history.Undo().has_value());
    CHECK_NEAR(node.value.cameraData.position.x, 100.0f, 0.0f);
    CHECK(!gizmo.history.GetUndoLabel());
    CHECK(gizmo.history.Redo().has_value());
    CHECK_NEAR(node.value.cameraData.position.x, 150.0f, 1e-3f);

    // a second drag is a step of its own
    gizmo.Frame(true, glm::vec3(0, 0, 7));
    gizmo.Frame(false);
    CHECK(gizmo.history.Undo().has_value());
    CHECK_NEAR(node.value.cameraData.position.z, 30.0f, 0.0f);
    CHECK_NEAR(node.value.cameraData.position.x, 150.0f, 1e-3f);
    CHECK(gizmo.history.Undo().has_value());
    CHECK_NEAR(node.value.cameraData.position.x, 100.0f, 0.0f);
}

TEST("a node's turn goes back from the frame that grabbed it, which already turned it")
{
    auto node = Node(20000, {100, 200, 30}, {10, 45, 0}, 80, KeyInterpolation::Auto);
    NodeGizmo gizmo{node};
    // (the rotation gizmo takes the mouse's delta on the frame it grabs the ring)
    gizmo.Frame(true, glm::vec3(0.0f), glm::vec3(0, 4, 0));
    gizmo.Frame(true, glm::vec3(0.0f), glm::vec3(0, 6, 0));
    gizmo.Frame(false);
    CHECK(gizmo.labels.size() == 1 && gizmo.labels.front() == "turn");
    CHECK_NEAR(node.value.cameraData.rotation.y, 55.0f, 1e-3f);
    CHECK(gizmo.history.Undo().has_value());
    CHECK_NEAR(node.value.cameraData.rotation.y, 45.0f, 0.0f);
}

TEST("a click on a node's gizmo that moves nothing is no step, and leaves the node's angles as they were")
{
    auto node = Node(20000, {100, 200, 30}, {10, 200, 0}, 80, KeyInterpolation::Auto);
    NodeGizmo gizmo{node};
    gizmo.Frame(true);
    gizmo.Frame(false);
    CHECK(!gizmo.history.GetUndoLabel());

    // the gizmo writes the same turn another way round: yaw 200 as -160
    gizmo.Frame(true, glm::vec3(0.0f), glm::vec3(0, -360, 0));
    gizmo.Frame(false);
    CHECK(!gizmo.history.GetUndoLabel());
    CHECK_NEAR(node.value.cameraData.rotation.y, 200.0f, 0.0f);
}

TEST("the gizmo going to another key ends the drag of the first, and takes the second as it is")
{
    auto first = Node(20000, {0, 0, 0}, {0, 0, 0}, 80, KeyInterpolation::Auto);
    auto second = Node(21000, {500, 0, 0}, {0, 0, 0}, 80, KeyInterpolation::Auto);
    KeyframeLogic::KeyDrag drag;
    CHECK(!drag.Frame(&first));
    CHECK(!drag.Frame(&first));
    const auto ended = drag.Frame(&second);
    CHECK(ended && ended->id == first.id);
    const auto last = drag.Frame(nullptr);
    CHECK(last && last->id == second.id && last->before.cameraData.position.x == 500.0f);
    CHECK(!drag.Frame(nullptr));
}

TEST("two values of a node are the same camera however its angles are written, and not by a hair more")
{
    const Types::CameraData camera{{100, 200, 30}, {10, 200, 5}, 80};
    auto same = camera;
    same.rotation.y = -160;
    CHECK(KeyframeLogic::IsSameCamera(camera, same));
    // over the top: pitch 170, yaw + 180, roll + 180 look the same way
    const Types::CameraData over{{0, 0, 0}, {170, 180, 180}, 80};
    const Types::CameraData under{{0, 0, 0}, {10, 0, 0}, 80};
    CHECK(KeyframeLogic::IsSameCamera(over, under));

    // (the gizmo's own rounding on a click, 0.002 units in game)
    auto rounded = camera;
    rounded.position.x += 0.002f;
    CHECK(KeyframeLogic::IsSameCamera(camera, rounded));
    auto moved = camera;
    moved.position.z += 0.5f;
    CHECK(!KeyframeLogic::IsSameCamera(camera, moved));
    auto turned = camera;
    turned.rotation.y += 0.5f;
    CHECK(!KeyframeLogic::IsSameCamera(camera, turned));
    auto zoomed = camera;
    zoomed.fov += 1.0f;
    CHECK(!KeyframeLogic::IsSameCamera(camera, zoomed));

    CHECK(KeyframeLogic::GetNodeDragVerb(camera, moved) == "move");
    CHECK(KeyframeLogic::GetNodeDragVerb(camera, turned) == "turn");
}
