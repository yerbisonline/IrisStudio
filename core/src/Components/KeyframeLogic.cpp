#include "StdInclude.hpp"
#include "KeyframeLogic.hpp"

#include "RuntimeProperties.hpp"

#include "glm/gtx/euler_angles.hpp"

namespace IWXMVM::KeyframeLogic
{
    namespace
    {
        // the campath's own curves and the eases' shape that leaves an end without one unbent (SetPathCurves)
        std::atomic<bool> pathCurves = true;
    }  // namespace

    bool IsAngle(Types::KeyframeValueType type, std::uint32_t index)
    {
        return type == Types::KeyframeValueType::CameraData && index >= 3 && index <= 5;
    }

    void Unwrap(std::span<float> degrees)
    {
        for (std::size_t i = 1; i < degrees.size(); ++i)
        {
            const auto turns = std::round((degrees[i] - degrees[i - 1]) / 360.0f);
            degrees[i] -= turns * 360.0f;
        }
    }

    std::vector<float> GetValues(std::span<const Types::Keyframe> keys, std::uint32_t index, bool angle)
    {
        std::vector<float> values;
        values.reserve(keys.size());
        for (const auto& key : keys)
            values.push_back(key.value.GetByIndex(index));
        if (angle)
            Unwrap(values);
        return values;
    }

    double Ease(double fraction, float easeOut, float easeIn)
    {
        const double t = std::clamp(fraction, 0.0, 1.0);
        const double out = std::clamp(easeOut, 0.0f, 1.0f);
        const double in = std::clamp(easeIn, 0.0f, 1.0f);
        if (!pathCurves)
        {
            // (the shape eases had at first: a cubic, which also bends the stretch's other end)
            const double t2 = t * t, t3 = t2 * t;
            return (t3 - 2 * t2 + t) * (1.0 - out) + (-2 * t3 + 3 * t2) + (t3 - t2) * (1.0 - in);
        }
        // the fraction, its slope at each end 1 less that end's ease, and unbent at an end without one: an ease out of
        // the first node leaves the second node's speed and acceleration as the curve has them, so no node but the
        // eased one shows. It never goes back.
        const double rest = 1.0 - t;
        return t - out * t * rest * rest * rest + in * t * t * t * rest;
    }

    double Uneased(double eased, float easeOut, float easeIn)
    {
        double low = 0.0, high = 1.0;
        for (int i = 0; i < 50; ++i)
        {
            const double middle = (low + high) / 2;
            (Ease(middle, easeOut, easeIn) < eased ? low : high) = middle;
        }
        return (low + high) / 2;
    }

    bool IsPath(std::span<const Types::Keyframe> keys)
    {
        return pathCurves && !keys.empty() &&
               keys.front().property.get().type == Types::KeyframeablePropertyType::CampathCamera;
    }

    void SetPathCurves(bool on)
    {
        pathCurves = on;
    }

    double GetPathSlope(std::span<const Types::Keyframe> keys, std::span<const float> values, std::size_t key)
    {
        const auto n = keys.size();
        if (n < 2 || key >= n)
            return 0.0;
        const auto secant = [&](std::size_t i) {
            const double span = static_cast<double>(keys[i + 1].tick) - keys[i].tick;
            return span > 0.0 ? (values[i + 1] - values[i]) / span : 0.0;
        };
        if (n == 2)
            return secant(0);
        // (unbent at the ends: a cubic from p0 to p1 with slopes m0 and m1 has no second derivative at p0 when
        // m0 = (3 * secant - m1) / 2, and at p1 when m1 = (3 * secant - m0) / 2)
        if (key == 0)
            return 1.5 * secant(0) - 0.5 * GetPathSlope(keys, values, 1);
        if (key + 1 == n)
            return 1.5 * secant(n - 2) - 0.5 * GetPathSlope(keys, values, n - 2);
        const double before = static_cast<double>(keys[key].tick) - keys[key - 1].tick;
        const double after = static_cast<double>(keys[key + 1].tick) - keys[key].tick;
        if (before <= 0.0 || after <= 0.0)
            return 0.0;
        return (secant(key - 1) * after + secant(key) * before) / (before + after);
    }

    double GetSmoothSlope(std::span<const Types::Keyframe> keys, std::span<const float> values, std::size_t key)
    {
        if (key == 0 || key + 1 >= keys.size())
            return 0.0;
        const double before = static_cast<double>(keys[key].tick) - keys[key - 1].tick;
        const double after = static_cast<double>(keys[key + 1].tick) - keys[key].tick;
        if (before <= 0.0 || after <= 0.0)
            return 0.0;
        const double slopeBefore = (values[key] - values[key - 1]) / before;
        const double slopeAfter = (values[key + 1] - values[key]) / after;
        // (where the values turn, or stay)
        if (slopeBefore * slopeAfter <= 0.0)
            return 0.0;
        const double slope = (slopeBefore * after + slopeAfter * before) / (before + after);
        const double limit = 3.0 * std::min(std::abs(slopeBefore), std::abs(slopeAfter));
        return std::clamp(slope, -limit, limit);
    }

    float InterpolateLinear(std::span<const float> ticks, std::span<const float> values, float tick)
    {
        if (values.size() == 1)
            return values.front();

        std::size_t p0 = 0, p1 = 1;
        for (std::size_t i = 0; i < values.size() - 1; i++)
        {
            if (tick >= ticks[i] && tick <= ticks[i + 1])
            {
                p0 = i;
                p1 = i + 1;
                break;
            }
        }
        const float t = (tick - ticks[p0]) / (ticks[p1] - ticks[p0]);
        return (1.0f - t) * values[p0] + t * values[p1];
    }

    // Copyright (c) by NUMERICAL RECIPES IN C: THE ART OF SCIENTIFIC COMPUTING (ISBN 0-521-43108-5)
    // Modified. Thank you to dtugend for finding this! (Moved here from MathUtils as it was, for "Auto".)
    float InterpolateCubicSpline(std::span<const float> ticks, std::span<const float> values, float tick, bool atRest)
    {
        const size_t n = values.size();
        if (n < 2)
            return n == 1 ? values.front() : 0.0f;

        constexpr int32_t MAX_NODES = 256;
        if (n > MAX_NODES)
            return values.back();

        float y2[MAX_NODES];  // second derivatives
        float u[MAX_NODES];

        // (at rest: the slope at the first key is 0; else its second derivative is, and the same at the last key)
        y2[0] = atRest ? -0.5f : 0.0f;
        u[0] = atRest ? (3.0f / (ticks[1] - ticks[0])) * ((values[1] - values[0]) / (ticks[1] - ticks[0])) : 0.0f;

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
            u[i] = (nextValue - currValue) / (nextTick - currTick) - (currValue - prevValue) / (currTick - prevTick);
            u[i] = (6.0f * u[i] / (nextTick - prevTick) - sig * u[i - 1]) / p;
        }

        auto qn = atRest ? 0.5f : 0.0f;
        auto un = atRest ? (3.0f / (ticks[n - 1] - ticks[n - 2])) *
                               (0.0f - (values[n - 1] - values[n - 2]) / (ticks[n - 1] - ticks[n - 2]))
                         : 0.0f;

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

    float Evaluate(std::span<const Types::Keyframe> keys, std::uint32_t index, bool angle, double tick, bool eased)
    {
        if (keys.empty())
            return 0.0f;
        return Evaluate(keys, GetValues(keys, index, angle), tick, eased);
    }

    float Evaluate(std::span<const Types::Keyframe> keys, std::span<const float> values, double tick, bool eased)
    {
        if (keys.empty() || values.size() != keys.size())
            return 0.0f;
        if (tick <= keys.front().tick)
            return values.front();
        if (tick >= keys.back().tick)
            return values.back();

        const auto next = std::upper_bound(keys.begin(), keys.end(), tick,
                                           [](double at, const Types::Keyframe& key) { return at < key.tick; });
        const auto i = static_cast<std::size_t>(next - keys.begin()) - 1;
        const auto& key = keys[i];
        const auto& after = keys[i + 1];
        if (key.interpolation == Types::KeyInterpolation::Hold)
            return values[i];

        const double span = static_cast<double>(after.tick) - key.tick;
        const bool easing = eased && (key.easeOut > 0.0f || after.easeIn > 0.0f);
        double fraction = (tick - key.tick) / span;
        if (easing)
            fraction = Ease(fraction, key.easeOut, after.easeIn);

        switch (key.interpolation)
        {
            case Types::KeyInterpolation::Auto:
            {
                // (in floats, as it was, so the Look's keys of older files look exactly as they did; the campath's
                // spline goes through its ends and curves from three nodes, IsPath)
                std::vector<float> ticks;
                ticks.reserve(keys.size());
                for (const auto& each : keys)
                    ticks.push_back(static_cast<float>(each.tick));
                const auto at = easing ? static_cast<float>(key.tick + fraction * span) : static_cast<float>(tick);
                if (IsPath(keys))
                    return InterpolateCubicSpline(ticks, values, at, false);
                return keys.size() < 4 ? InterpolateLinear(ticks, values, at)
                                       : InterpolateCubicSpline(ticks, values, at);
            }
            case Types::KeyInterpolation::Linear:
                return static_cast<float>(values[i] + (values[i + 1] - values[i]) * fraction);
            default:
            {
                // a cubic Hermite curve with the slopes Smooth gives the two keys (the campath's own, IsPath)
                const auto slope = IsPath(keys) ? GetPathSlope : GetSmoothSlope;
                const double t = fraction, t2 = t * t, t3 = t2 * t;
                const double leaving = slope(keys, values, i) * span;
                const double arriving = slope(keys, values, i + 1) * span;
                return static_cast<float>((2 * t3 - 3 * t2 + 1) * values[i] + (t3 - 2 * t2 + t) * leaving +
                                          (-2 * t3 + 3 * t2) * values[i + 1] + (t3 - t2) * arriving);
            }
        }
    }

    Types::KeyframeValue Evaluate(Types::KeyframeValueType type, std::span<const Types::Keyframe> keys, double tick,
                                  bool eased)
    {
        const auto value = [&](std::uint32_t index) {
            return Evaluate(keys, index, IsAngle(type, index), tick, eased);
        };
        switch (type)
        {
            case Types::KeyframeValueType::FloatingPoint:
                return Types::KeyframeValue(value(0));
            case Types::KeyframeValueType::Vector3:
                return Types::KeyframeValue(glm::vec3(value(0), value(1), value(2)));
            case Types::KeyframeValueType::CameraData:
                return Types::KeyframeValue(Types::CameraData{glm::vec3(value(0), value(1), value(2)),
                                                              glm::vec3(value(3), value(4), value(5)), value(6)});
        }
        return Types::KeyframeValue(0.0f);
    }

    Types::KeyInterpolation GetDefaultInterpolation(Types::KeyframeablePropertyType type)
    {
        if (IsStepped(type))
            return Types::KeyInterpolation::Hold;
        return type == Types::KeyframeablePropertyType::CampathCamera ? Types::KeyInterpolation::Auto
                                                                      : Types::KeyInterpolation::Smooth;
    }

    std::optional<Types::KeyInterpolation> GetSharedInterpolation(std::span<const Types::Keyframe> keys)
    {
        if (keys.empty())
            return std::nullopt;
        const auto first = keys.front().interpolation;
        if (!std::ranges::all_of(keys, [first](const Types::Keyframe& key) { return key.interpolation == first; }))
            return std::nullopt;
        return first;
    }

    PathProfile MakeProfile(std::span<const PathSample> samples)
    {
        PathProfile profile;
        std::vector<PathSample> kept;
        for (const auto& sample : samples)
        {
            if (kept.empty() || sample.tick > kept.back().tick)
                kept.push_back(sample);
        }
        profile.frames = kept.size();
        if (kept.size() < 2)
            return profile;

        // each frame to the next: its velocity, and the tick halfway
        struct Step
        {
            glm::dvec3 velocity;
            double speed, tick;
        };
        std::vector<Step> steps;
        for (std::size_t i = 0; i + 1 < kept.size(); ++i)
        {
            const double seconds = (kept[i + 1].tick - kept[i].tick) / 1000.0;
            const auto velocity = glm::dvec3(kept[i + 1].position - kept[i].position) / seconds;
            steps.push_back({velocity, glm::length(velocity), (kept[i].tick + kept[i + 1].tick) / 2.0});
        }
        profile.first = steps.front().speed;
        profile.last = steps.back().speed;
        profile.least = profile.most = profile.first;
        for (const auto& step : steps)
        {
            profile.least = std::min(profile.least, step.speed);
            profile.most = std::max(profile.most, step.speed);
        }
        std::optional<double> acceleration;
        for (std::size_t i = 0; i + 1 < steps.size(); ++i)
        {
            const auto& a = steps[i];
            const auto& b = steps[i + 1];
            if (a.speed > 1e-6 && b.speed > 1e-6)
            {
                const auto turn = glm::degrees(
                    std::acos(std::clamp(glm::dot(a.velocity, b.velocity) / (a.speed * b.speed), -1.0, 1.0)));
                if (turn > profile.turn)
                {
                    profile.turn = turn;
                    profile.turnTick = (a.tick + b.tick) / 2.0;
                }
            }
            const double now = (b.speed - a.speed) / ((b.tick - a.tick) / 1000.0);
            if (acceleration && std::abs(now - *acceleration) > profile.accelerationStep)
            {
                profile.accelerationStep = std::abs(now - *acceleration);
                profile.accelerationStepTick = a.tick;
            }
            acceleration = now;
        }
        return profile;
    }

    Types::KeyInterpolation GetNewKeyInterpolation(std::span<const Types::Keyframe> keys, std::uint32_t tick,
                                                   Types::KeyframeablePropertyType type)
    {
        if (keys.empty() || IsStepped(type))
            return GetDefaultInterpolation(type);
        const Types::Keyframe* before = nullptr;
        for (const auto& key : keys)
        {
            if (key.tick <= tick)
                before = &key;
        }
        return (before ? *before : keys.front()).interpolation;
    }

    bool IsStepped(Types::KeyframeablePropertyType type)
    {
        using Type = Types::KeyframeablePropertyType;
        if (const auto runtime = Components::RuntimeProperties::Find(type))
            return runtime->steps != Components::RuntimeProperties::Steps::None;
        return type == Type::FilmtweakEnabled || type == Type::FilmtweakInvert || type == Type::DepthOfFieldEnabled;
    }

    float GetStepValue(float value)
    {
        return value >= 0.5f ? 1.0f : 0.0f;
    }

    float GetStepValue(Types::KeyframeablePropertyType type, float value)
    {
        if (const auto runtime = Components::RuntimeProperties::Find(type);
            runtime && runtime->steps == Components::RuntimeProperties::Steps::Whole)
            return std::round(value);
        return GetStepValue(value);
    }

    Types::Keyframe Normalize(Types::Keyframe key, Types::KeyframeablePropertyType type)
    {
        if (!IsStepped(type))
            return key;
        key.interpolation = Types::KeyInterpolation::Hold;
        key.easeIn = 0.0f;
        key.easeOut = 0.0f;
        key.value.floatingPoint = GetStepValue(type, key.value.floatingPoint);
        return key;
    }

    bool ApplyAutoEase(std::span<Types::Keyframe> nodes, float start, float end, Types::AutoEaseNodes which)
    {
        const bool all = which == Types::AutoEaseNodes::AllNodes;
        bool changed = false;
        const auto set = [&](float& ease, float amount) {
            amount = std::clamp(amount, 0.0f, 1.0f);
            changed = changed || ease != amount;
            ease = amount;
        };
        for (std::size_t i = 0; i < nodes.size(); ++i)
        {
            auto& node = nodes[i];
            const bool first = i == 0, last = i + 1 == nodes.size();
            if (node.autoEaseOut)
                set(node.easeOut, first || (all && !last) ? start : 0.0f);
            if (node.autoEaseIn)
                set(node.easeIn, last || (all && !first) ? end : 0.0f);
        }
        return changed;
    }

    void KeepChangedEases(const Types::Keyframe& before, Types::Keyframe& after)
    {
        if (after.easeIn != before.easeIn)
            after.autoEaseIn = false;
        if (after.easeOut != before.easeOut)
            after.autoEaseOut = false;
    }

    double Retiming::GetLength() const
    {
        return lengths.empty() ? 0.0 : lengths.back();
    }

    Retiming MakeRetiming(std::span<const Types::Keyframe> nodes, std::size_t samplesPerNode)
    {
        Retiming retiming;
        if (nodes.size() < 2)
            return retiming;
        retiming.start = nodes.front().tick;
        retiming.end = nodes.back().tick;
        retiming.easeOut = nodes.front().easeOut;
        retiming.easeIn = nodes.back().easeIn;

        const std::array values = {GetValues(nodes, 0, false), GetValues(nodes, 1, false), GetValues(nodes, 2, false)};
        const auto position = [&](double tick) {
            return glm::vec3(Evaluate(nodes, values[0], tick, false), Evaluate(nodes, values[1], tick, false),
                             Evaluate(nodes, values[2], tick, false));
        };
        samplesPerNode = std::max<std::size_t>(samplesPerNode, 1);
        auto last = position(retiming.start);
        double length = 0.0;
        retiming.ticks.push_back(retiming.start);
        retiming.lengths.push_back(0.0);
        for (std::size_t i = 0; i + 1 < nodes.size(); ++i)
        {
            const double from = nodes[i].tick, to = nodes[i + 1].tick;
            for (std::size_t s = 1; s <= samplesPerNode; ++s)
            {
                const double tick = from + (to - from) * s / samplesPerNode;
                const auto here = position(tick);
                length += glm::distance(last, here);
                last = here;
                retiming.ticks.push_back(tick);
                retiming.lengths.push_back(length);
            }
        }
        return retiming;
    }

    double Retime(const Retiming& retiming, double tick)
    {
        if (retiming.ticks.size() < 2 || retiming.end <= retiming.start || tick <= retiming.start ||
            tick >= retiming.end)
            return tick;
        const double fraction =
            Ease((tick - retiming.start) / (retiming.end - retiming.start), retiming.easeOut, retiming.easeIn);
        const double total = retiming.GetLength();
        if (total <= 0.0)
            return retiming.start + fraction * (retiming.end - retiming.start);

        // the sample the camera has passed, and how far it is to the next
        const double length = fraction * total;
        const auto next = std::upper_bound(retiming.lengths.begin(), retiming.lengths.end(), length);
        if (next == retiming.lengths.end())
            return retiming.end;
        const auto j = static_cast<std::size_t>(next - retiming.lengths.begin());
        const double from = retiming.lengths[j - 1], to = retiming.lengths[j];
        const double along = to > from ? (length - from) / (to - from) : 0.0;
        return retiming.ticks[j - 1] + along * (retiming.ticks[j] - retiming.ticks[j - 1]);
    }

    double GetPassingTick(const Retiming& retiming, double pathTick)
    {
        const double total = retiming.GetLength();
        if (retiming.ticks.size() < 2 || retiming.end <= retiming.start || total <= 0.0)
            return pathTick;
        if (pathTick <= retiming.start)
            return retiming.start;
        if (pathTick >= retiming.end)
            return retiming.end;
        const auto next = std::upper_bound(retiming.ticks.begin(), retiming.ticks.end(), pathTick);
        const auto j = static_cast<std::size_t>(next - retiming.ticks.begin());
        const double along = (pathTick - retiming.ticks[j - 1]) / (retiming.ticks[j] - retiming.ticks[j - 1]);
        const double length = retiming.lengths[j - 1] + along * (retiming.lengths[j] - retiming.lengths[j - 1]);
        const double fraction = Uneased(length / total, retiming.easeOut, retiming.easeIn);
        return retiming.start + fraction * (retiming.end - retiming.start);
    }

    Types::KeyframeValue EvaluateRetimed(const Retiming& retiming, std::span<const Types::Keyframe> nodes, double tick)
    {
        return Evaluate(Types::KeyframeValueType::CameraData, nodes, Retime(retiming, tick), false);
    }

    std::optional<KeyDrag::Taken> KeyDrag::Frame(const Types::Keyframe* held)
    {
        std::optional<Taken> ended;
        if (taken && (!held || held->id != taken->id))
        {
            ended = taken;
            taken.reset();
        }
        if (held && !taken)
            taken = Taken{held->id, held->value};
        return ended;
    }

    namespace
    {
        // below anything that shows: the gizmo's own rounding moves a node 0.002 units on a click (float at 1700 units)
        constexpr float SAME_PLACE = 0.01f;
    }  // namespace

    bool IsSameCamera(const Types::CameraData& a, const Types::CameraData& b)
    {
        constexpr float AXES = 0.0001f;
        const auto turn = [](const glm::vec3& rotation) {
            return glm::mat3(glm::eulerAngleZYX(glm::radians(rotation.y), glm::radians(rotation.x),
                                                glm::radians(rotation.z)));
        };
        if (glm::distance(a.position, b.position) > SAME_PLACE || std::abs(a.fov - b.fov) > SAME_PLACE)
            return false;
        const auto x = turn(a.rotation), y = turn(b.rotation);
        for (int i = 0; i < 3; ++i)
        {
            if (glm::distance(x[i], y[i]) > AXES)
                return false;
        }
        return true;
    }

    std::string_view GetNodeDragVerb(const Types::CameraData& before, const Types::CameraData& after)
    {
        return glm::distance(before.position, after.position) > SAME_PLACE ? "move" : "turn";
    }
}  // namespace IWXMVM::KeyframeLogic
