#ifndef PATROL_CONTROL_ASYNC_SERVO_H_
#define PATROL_CONTROL_ASYNC_SERVO_H_

#include "patrol_control/drop_action.h"
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <array>
#include <cmath>

namespace patrol_control {

// One in-flight call, no queue. Slots remain locked unless the fenced proxy
// positively proves NOT_STARTED for that exact, completed RPC.
// Only this worker's private state crosses threads; controller state does not.
class AsyncServo {
public:
    enum class Status { kCompleted, kCancelled, kTimedOut };
    struct Completion {
        std::uint64_t action = 0;
        int slot = 0;
        Status status = Status::kCancelled;
        DropActionResult result = DropActionResult::kServiceCallFailed;
        bool invocation_started = false;
    };
    using Execute = std::function<DropActionResult(int)>;
    using Clock = std::chrono::steady_clock;

    AsyncServo() = default;
    AsyncServo(const AsyncServo&) = delete;
    AsyncServo& operator=(const AsyncServo&) = delete;
    ~AsyncServo() { shutdown(); }

    bool submit(std::uint64_t action, int slot, double timeout_sec, Execute execute) {
        if (stopped_ || !action || slot < 1 || slot > 3 ||
            !execute || !std::isfinite(timeout_sec) || timeout_sec <= 0.0) return false;
        // The result topic/context may close the old action before its RPC
        // returns. Reap its fenced NOT_STARTED proof independently of poll.
        if (job_) releaseNotStarted(job_->action, job_->slot);
        if (locked_[slot - 1] || owners_[slot - 1] == action) return false;
        if (job_) {
            std::lock_guard<std::mutex> lock(job_->mutex);
            if (!job_->done) return false;
        }
        // A finished call can be reaped; never join a running call here.
        if (thread_.joinable()) thread_.join();
        auto job = std::make_shared<Job>();
        job->action = action;
        job->slot = slot;
        job->deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(timeout_sec));
        job_ = job;
        locked_[slot - 1] = true;
        owners_[slot - 1] = action;
        try {
            thread_ = std::thread([job, execute]() {
                {
                    std::lock_guard<std::mutex> lock(job->mutex);
                    if (job->cancelled || Clock::now() >= job->deadline) {
                        job->timed_out = Clock::now() >= job->deadline;
                        job->done = true;
                        return;
                    }
                    // Linearization point: cancel after this point cannot undo
                    // an already admitted physical call and must never retry it.
                    job->started = true;
                }
                DropActionResult result = DropActionResult::kServiceCallFailed;
                try { result = execute(job->slot); } catch (...) {}
                std::lock_guard<std::mutex> lock(job->mutex);
                job->result = result;
                job->finished_at = Clock::now();
                job->done = true;
            });
        } catch (...) {
            // No thread and hence no actuator invocation was created.
            locked_[slot - 1] = false;
            job_.reset();
            return false;
        }
        return true;
    }

    bool poll(std::uint64_t action, int slot, Completion* output) {
        if (!job_ || !output) return false;
        std::lock_guard<std::mutex> lock(job_->mutex);
        if (job_->action != action || job_->slot != slot || job_->consumed) return false;
        const bool timed_out = job_->timed_out ||
            (job_->done ? job_->finished_at > job_->deadline
                        : Clock::now() >= job_->deadline);
        if (!job_->done && !job_->cancelled && !timed_out) return false;
        job_->timed_out = timed_out;
        job_->consumed = true;
        output->action = job_->action;
        output->slot = job_->slot;
        output->invocation_started = job_->started;
        output->result = job_->result;
        output->status = timed_out ? Status::kTimedOut :
            (job_->cancelled ? Status::kCancelled : Status::kCompleted);
        return true;
    }

    // Called only on the owner callback thread. A proof never acknowledges a
    // release and never enables retry of the same local action generation.
    bool releaseNotStarted(std::uint64_t action, int slot) {
        if (stopped_ || !job_ || slot < 1 || slot > 3) return false;
        std::lock_guard<std::mutex> lock(job_->mutex);
        if (job_->action != action || job_->slot != slot ||
            owners_[slot - 1] != action || !locked_[slot - 1] ||
            !job_->done || job_->result != DropActionResult::kNotStarted) return false;
        // Cancel/timeout means the caller stopped waiting, not that the proxy
        // lost its ability to prove zero raw calls. Only the classified terminal
        // RPC proof can release the slot; unknown/started results never do.
        locked_[slot - 1] = false;
        return true;
    }

    void cancel() {
        if (!job_) return;
        std::lock_guard<std::mutex> lock(job_->mutex);
        job_->cancelled = true;
    }

    void shutdown() {
        if (stopped_) return;
        stopped_ = true;
        cancel();
        if (!thread_.joinable()) return;
        bool done;
        {
            std::lock_guard<std::mutex> lock(job_->mutex);
            done = job_->done;
        }
        if (done) thread_.join();
        else thread_.detach();
        // ROS1 synchronous call has no guaranteed cancellable timeout. A hung
        // call owns its client and shared Job after destruction, never `this`.
        // No further calls/threads can be spawned by this stopped worker.
    }

private:
    struct Job {
        std::mutex mutex;
        std::uint64_t action = 0;
        int slot = 0;
        Clock::time_point deadline, finished_at;
        bool started = false, done = false, cancelled = false;
        bool consumed = false, timed_out = false;
        DropActionResult result = DropActionResult::kServiceCallFailed;
    };
    std::shared_ptr<Job> job_;
    std::thread thread_;
    std::array<bool, 3> locked_{{false, false, false}};
    std::array<std::uint64_t, 3> owners_{{0, 0, 0}};
    bool stopped_ = false;
};

} // namespace patrol_control
#endif
