#ifndef AUTORACE_ASYNC_LATEST_WORKER_HPP
#define AUTORACE_ASYNC_LATEST_WORKER_HPP

#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace autorace_async {

// One in-flight job and at most one pending job. The handler never owns the
// mailbox mutex. Explicit revisions reject delayed submissions from producers
// such as a watchdog and a perception worker racing to submit their results.
template<class T> class LatestWorker {
public:
  using Value = std::shared_ptr<const T>;
  using Handler = std::function<void(const T &)>;
  using ErrorHandler = std::function<void(std::exception_ptr)>;

  LatestWorker() = default;
  LatestWorker(const LatestWorker &) = delete;
  LatestWorker &operator=(const LatestWorker &) = delete;
  ~LatestWorker() { stop(); }

  void start(Handler handler, ErrorHandler error = {}) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (started_)
        throw std::logic_error("LatestWorker can only be started once");
      started_ = true;
      handler_ = std::move(handler);
      error_ = std::move(error);
    }
    worker_ = std::thread([this] { run(); });
  }

  bool submit(Value value, uint64_t revision = 0) {
    Value displaced;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!started_ || stopping_ || !value)
        return false;
      if (revision && revision <= revision_)
        return false;
      revision_ = revision ? revision : revision_ + 1;
      displaced = std::move(pending_);
      pending_ = std::move(value);
    }
    cv_.notify_one();
    return true;
  }

  void stop() {
    Value discarded;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      discarded = std::move(pending_);
    }
    cv_.notify_one();
    if (worker_.joinable())
      worker_.join();
  }

private:
  void run() {
    while (true) {
      Value value;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return stopping_ || pending_; });
        if (stopping_)
          return;
        value = std::move(pending_);
      }
      try {
        handler_(*value);
      } catch (...) {
        if (error_) {
          try { error_(std::current_exception()); } catch (...) {}
        }
      }
    }
  }

  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  Handler handler_;
  ErrorHandler error_;
  Value pending_;
  uint64_t revision_ = 0;
  bool started_ = false;
  bool stopping_ = false;
};

} // namespace autorace_async
#endif
