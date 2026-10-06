#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/time.h>
#include <thread>
#include <time.h>
#include <vector>
#include "../src/Arduino.h"

static const uint64_t origin = 987654321987654ULL;
static std::atomic<uint64_t> monotonicNs{origin};
static std::atomic<int64_t> wallNs{1791295200000000000LL};
static std::atomic<unsigned> wallReads{0};
static bool realClock = false;

extern "C" int __real_clock_gettime(clockid_t, struct timespec*);
extern "C" int __wrap_clock_gettime(clockid_t id, struct timespec* ts){
    if (realClock) return __real_clock_gettime(id, ts);
    assert(id == CLOCK_MONOTONIC || id == CLOCK_REALTIME);
    const uint64_t ns = id == CLOCK_MONOTONIC ? monotonicNs.load() : wallNs.load();
    if (id == CLOCK_REALTIME) ++wallReads;
    ts->tv_sec = ns / 1000000000ULL;
    ts->tv_nsec = ns % 1000000000ULL;
    return 0;
}

extern "C" int __wrap_gettimeofday(struct timeval* tv, void*){
    ++wallReads;
    const int64_t ns = wallNs.load();
    tv->tv_sec = ns / 1000000000LL;
    tv->tv_usec = ns % 1000000000LL / 1000;
    return 0;
}

int main(int argc, char** argv){
    if (argc > 1 && std::strcmp(argv[1], "--real") == 0){
        realClock = true;
        const auto before = std::chrono::steady_clock::now();
        const auto us = micros();
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        const auto elapsed = micros() - us;
        const auto reference = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - before).count();
        assert(elapsed >= 20000 && elapsed <= static_cast<uint64_t>(reference) + 1000);
        assert(millis() >= 20);
        assert(wallReads == 0);
        std::puts("PASS: production timers advance with the real Linux monotonic clock");
        return 0;
    }

    // Concurrent first use must initialize one shared epoch for both APIs.
    std::vector<std::thread> threads;
    for (unsigned i = 0; i < 8; ++i) threads.emplace_back([i]{
        for (unsigned n = 0; n < 1000; ++n){
            assert((i % 2 ? millis() : micros()) == 0);
        }
    });
    for (auto& thread : threads) thread.join();

    monotonicNs = origin + 123456789ULL;
    assert(micros() == 123456 && millis() == 123);
    const unsigned long gpsDeadline = millis() + 3000;
    const unsigned long previous = millis();

    // Reproduce the logged +2281 s NTP step, backwards steps and large corrections.
    for (int64_t seconds : {2281LL, -3869LL, 86400LL, -172800LL}){
        wallNs += seconds * 1000000000LL;
        assert(micros() == 123456 && millis() == previous);
        assert(millis() < gpsDeadline);
    }
    monotonicNs += 20000000ULL;
    assert(millis() - previous == 20); // PID interval remains 20 ms.
    monotonicNs += 2980000000ULL;
    assert(millis() == gpsDeadline); // A real 3 s timeout still expires.

    // Preserve unsigned-long width/wrap behavior; no premature 32-bit arithmetic.
    for (uint64_t ns : {4294967295ULL * 1000, 4294967296ULL * 1000,
                        4294967295ULL * 1000000, 4294967296ULL * 1000000,
                        100ULL * 86400 * 1000000000}){
        monotonicNs = origin + ns;
        assert(micros() == static_cast<unsigned long>(ns / 1000));
        assert(millis() == static_cast<unsigned long>(ns / 1000000));
    }
    assert(wallReads == 0);
    std::puts("PASS: shared epoch, concurrent startup, wall-clock jumps, GPS/PID deadlines, long uptime");
}
