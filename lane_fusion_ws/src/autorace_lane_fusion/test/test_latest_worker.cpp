#include "autorace_lane_fusion/latest_worker.hpp"
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <vector>

using autorace_async::LatestWorker;
using namespace std::chrono_literals;

struct Gate {
  std::mutex mutex;
  std::condition_variable cv;
  bool ready = false;
  void wait() {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [this] { return ready; });
  }
  void open() {
    { std::lock_guard<std::mutex> lock(mutex); ready = true; }
    cv.notify_all();
  }
};

void require(bool ok, const char *what) {
  if (!ok) throw std::runtime_error(what);
}

void latest_and_revisions() {
  Gate gate;
  std::promise<void> entered, delivered;
  auto first = entered.get_future(), last = delivered.get_future();
  std::vector<int> values;
  LatestWorker<int> worker;
  worker.start([&](const int &v) {
    values.push_back(v);
    if (v == 1) { entered.set_value(); gate.wait(); }
    if (v == 1000) delivered.set_value();
  });
  worker.submit(std::make_shared<int>(1), 1);
  bool began = first.wait_for(2s) == std::future_status::ready;
  for (int i = 2; i <= 1000; ++i) worker.submit(std::make_shared<int>(i), i);
  bool stale_rejected = !worker.submit(std::make_shared<int>(999), 999);
  bool still_blocked = last.wait_for(20ms) != std::future_status::ready;
  gate.open();
  bool finished = last.wait_for(2s) == std::future_status::ready;
  worker.stop();
  require(began && finished && still_blocked, "blocked handler prevented producer progress");
  require(stale_rejected, "older watchdog/result revision accepted");
  require(values == std::vector<int>({1, 1000}), "pending jobs accumulated instead of replacing");
}

void independent_streams() {
  Gate gate;
  std::promise<void> entered, delivered;
  auto began = entered.get_future(), finished = delivered.get_future();
  LatestWorker<int> debug, control;
  debug.start([&](const int &) { entered.set_value(); gate.wait(); });
  control.start([&](const int &) { delivered.set_value(); });
  debug.submit(std::make_shared<int>(1));
  bool waiting = began.wait_for(2s) == std::future_status::ready;
  control.submit(std::make_shared<int>(2));
  bool independent = finished.wait_for(1s) == std::future_status::ready;
  gate.open();
  debug.stop(); control.stop();
  require(waiting && independent, "debug/IMU transport blocked an independent stream");
}

void shutdown_discards_pending() {
  Gate gate;
  std::promise<void> entered, stopped;
  auto began = entered.get_future(), closed = stopped.get_future();
  std::vector<int> values;
  LatestWorker<int> worker;
  worker.start([&](const int &v) { values.push_back(v); entered.set_value(); gate.wait(); });
  worker.submit(std::make_shared<int>(1));
  bool waiting = began.wait_for(2s) == std::future_status::ready;
  worker.submit(std::make_shared<int>(2));
  std::thread closer([&] { worker.stop(); stopped.set_value(); });
  bool rejected = false;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!worker.submit(std::make_shared<int>(3))) { rejected = true; break; }
    std::this_thread::sleep_for(1ms);
  }
  bool joined = closed.wait_for(20ms) != std::future_status::ready;
  gate.open(); closer.join();
  require(waiting && rejected && joined, "shutdown failed to join/reject new output");
  require(values == std::vector<int>({1}), "shutdown transmitted a queued stale message");
}

void exceptions_do_not_terminate() {
  std::promise<void> failed, delivered;
  auto error = failed.get_future(), done = delivered.get_future();
  LatestWorker<int> worker;
  worker.start([&](const int &v) {
    if (v == 1) throw std::runtime_error("simulated DDS failure");
    delivered.set_value();
  }, [&](std::exception_ptr p) { if (p) failed.set_value(); });
  worker.submit(std::make_shared<int>(1));
  bool caught = error.wait_for(2s) == std::future_status::ready;
  worker.submit(std::make_shared<int>(2));
  bool resumed = done.wait_for(2s) == std::future_status::ready;
  worker.stop();
  require(caught && resumed, "publication exception terminated output worker");
}

int main() {
  try {
    latest_and_revisions(); independent_streams();
    shutdown_discards_pending(); exceptions_do_not_terminate();
    std::cout << "4 output concurrency tests passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
