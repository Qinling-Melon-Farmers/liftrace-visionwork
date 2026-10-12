#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

namespace patrol_control {

// One ordinary CommandBool RPC at a time; no worker touches controller state.
// Cancellation prevents calls not yet admitted. An already admitted ROS RPC
// cannot be recalled; its reply never proves actual disarm or completes LAND.
class AsyncLandingDisarm {
public:
    ~AsyncLandingDisarm() { cancel(); reap(); if (worker_.joinable()) worker_.detach(); }
    bool submit(std::function<void()> execute) {
        reap();
        if (worker_.joinable() || !execute) return false;
        auto job = std::make_shared<Job>();
        job_ = job;
        try {
            worker_ = std::thread([job, execute]() {
                if (!job->cancelled.load()) { try { execute(); } catch (...) {} }
                job->done.store(true);
            });
        } catch (...) { job_.reset(); return false; }
        return true;
    }
    void cancel() { if (job_) job_->cancelled.store(true); }
    void reap() { if (job_ && job_->done.load() && worker_.joinable()) worker_.join(); }
private:
    struct Job { std::atomic<bool> cancelled{false}, done{false}; };
    std::shared_ptr<Job> job_;
    std::thread worker_;
};

} // namespace patrol_control
