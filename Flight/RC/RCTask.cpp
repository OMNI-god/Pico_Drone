#include "RCTask.h"

#include "pico/time.h"

// ============================================================
// Constructor
// ============================================================

RCTask::RCTask(
    Elrs &elrs,
    CrsfParser &parser,
    RCInput &input,
    RCPublisher &publisher)
    : _elrs(elrs),
      _parser(parser),
      _input(input),
      _publisher(publisher)
{
}

// ============================================================
// Start
// ============================================================

bool RCTask::start(
    const char *taskName,
    std::uint32_t stackDepth,
    std::uint32_t priority)
{
    if (_taskHandle != nullptr)
    {
        return false;
    }

    const BaseType_t result =
        xTaskCreate(
            taskEntry,
            taskName,
            stackDepth,
            this,
            static_cast<UBaseType_t>(priority),
            &_taskHandle);

    return result == pdPASS;
}

// ============================================================
// Task entry
// ============================================================

void RCTask::taskEntry(void *parameter)
{
    auto *task =
        static_cast<RCTask *>(parameter);

    if (task == nullptr)
    {
        vTaskDelete(nullptr);
        return;
    }

    task->run();

    vTaskDelete(nullptr);
}

// ============================================================
// Main RC task
// ============================================================

void RCTask::run()
{
    constexpr TickType_t TaskPeriod =
        pdMS_TO_TICKS(1);

    TickType_t lastWakeTime =
        xTaskGetTickCount();

    _lastHealthUpdateUs =
        time_us_64();

    while (true)
    {
        processIncomingData();

        const std::uint64_t nowUs =
            time_us_64();

        const bool wasFailsafe =
            _input.isFailsafe();

        _input.updateFailsafe(nowUs);

        const bool isFailsafe =
            _input.isFailsafe();

        if (!wasFailsafe && isFailsafe)
        {
            _publisher.publish(
                _input.getState());
        }

        updateHealth(nowUs);

        vTaskDelayUntil(
            &lastWakeTime,
            TaskPeriod);
    }
}

// ============================================================
// Process UART
// ============================================================

void RCTask::processIncomingData()
{
    std::uint8_t byte = 0;

    while (_elrs.available())
    {
        if (_elrs.read(&byte, 1) != 1)
        {
            break;
        }

        ++_health.totalBytes;

        if (_parser.processByte(byte))
        {
            const std::uint64_t nowUs =
                time_us_64();

            processFrame(nowUs);
        }
    }
}

// ============================================================
// Process valid CRSF frame
// ============================================================

void RCTask::processFrame(
    std::uint64_t nowUs)
{
    _input.update(
        _parser.channels(),
        nowUs);

    const RCState &state =
        _input.getState();

    _publisher.publish(state);

    ++_health.totalFrames;

    _health.lastFrameTimeUs =
        nowUs;

    _health.frameAgeUs = 0;

    _health.connected = true;

    _health.failsafe =
        _input.isFailsafe();
}

// ============================================================
// Health monitoring
// ============================================================

void RCTask::updateHealth(
    std::uint64_t nowUs)
{
    if (_health.lastFrameTimeUs == 0)
    {
        _health.frameAgeUs = UINT64_MAX;
    }
    else
    {
        _health.frameAgeUs =
            nowUs -
            _health.lastFrameTimeUs;
    }

    const std::uint64_t elapsedUs =
        nowUs -
        _lastHealthUpdateUs;

    if (elapsedUs >= 1'000'000)
    {
        const std::uint32_t framesNow =
            _health.totalFrames;

        _health.framesLastSecond =
            framesNow -
            _framesAtLastHealthUpdate;

        _framesAtLastHealthUpdate =
            framesNow;

        _lastHealthUpdateUs =
            nowUs;
    }

    _health.connected =
        _health.lastFrameTimeUs != 0 &&
        _health.frameAgeUs <=
            RCHealth::MaximumFrameAgeUs;

    _health.failsafe =
        _input.isFailsafe();

    _health.healthy =
        _health.connected &&
        !_health.failsafe &&
        (_health.framesLastSecond >=
         RCHealth::MinimumFrameRate);
}

// ============================================================
// Health accessor
// ============================================================

const RCHealth &
RCTask::getHealth() const
{
    return _health;
}