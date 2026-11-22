#pragma once
#include "core.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>
namespace cone {
// One worker, one running job, one replaceable pending snapshot. Generations
// discard completed work belonging to an earlier mission, as in the Python worker.
template<class Metadata> class AsyncPlanner {
public:
struct Payload {
  Path left, right;
  Pairs pairs;
  Path measured;
  std::optional<double>width;
  Params parameters;
  double budget;
};
struct Result {
  Metadata metadata;
  Path path;
  Diag diagnostics;
};
private:
struct Job {
  size_t generation;
  Payload payload;
  Metadata metadata;
};
std::mutex mutex_;
std::condition_variable cv_;
std::thread worker_;
bool closing_ = false, running_ = false;
size_t generation_ = 0;
std::optional<Job>pending_;
std::optional<std::pair<size_t, Result> >result_;
void run()
{
  std::unique_lock<std::mutex>lock(mutex_);
  while (true) {
    cv_.wait(lock, [&] {
          return closing_||(!running_&&pending_.has_value());
        });
    if (closing_) return;
    auto job = std::move(*pending_);
    pending_.reset();
    running_ = true;
    lock.unlock();
    auto start = steady();
    auto p = job.payload.parameters;
    p["_planning_deadline"] = start + job.payload.budget;
    Result r{job.metadata, {}, {}};
    try{
      check_budget(p);
      r.path = unified_path(job.payload.left, job.payload.right, job.payload.pairs, job.payload.measured,
                            job.payload.width, p, &r.diagnostics);
    }catch (const BudgetExceeded&) {
      r.path = job.payload.measured;
      r.diagnostics["planning_budget"] = "exceeded; retained measured path";
    }catch (const std::exception&e) {
      r.path.clear();
      r.diagnostics["planner_error"] = e.what();
    } r.diagnostics["planning_ms"] = std::to_string(std::round((steady() - start) * 100000) / 100);
    lock.lock();
    result_ = std::make_pair(job.generation, std::move(r));
  }
}

public:
AsyncPlanner() : worker_([this] {
      run();
    }){}
~AsyncPlanner()
{
  {
    std::lock_guard<std::mutex>lock(mutex_);
    closing_ = true;
    pending_.reset();
  } cv_.notify_all();
  worker_.join();
}

void request(Payload p, Metadata m)
{
  std::lock_guard<std::mutex>lock(mutex_);
  pending_ = Job{generation_, std::move(p), std::move(m)};
  cv_.notify_one();
}

std::optional<Result>poll()
{
  std::lock_guard<std::mutex>lock(mutex_);
  if (!result_) return {};
  auto result = std::move(*result_);
  result_.reset();
  running_ = false;
  cv_.notify_one();
  if (result.first != generation_) return {};
  return std::move(result.second);
}

void reset()
{
  std::lock_guard<std::mutex>lock(mutex_);
  generation_++;
  pending_.reset();
}

};
} // namespace cone
