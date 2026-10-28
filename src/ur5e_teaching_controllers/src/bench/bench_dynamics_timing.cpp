// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Per-call wall time of gravity() and rnea() on both dynamics backends, single-threaded, no ROS
// graph and no simulator: the cost each backend adds to one 2 ms controller cycle at 500 Hz.
//
//   bench_dynamics_timing --urdf ur5e.urdf [--calls 100000] [--seed 1] [--output timing.csv]
//                         [--commit <sha>] [--build-type <type>] [--cpu <model>]
//
// For each backend the states (q, qd, qdd) are drawn up front, uniformly inside the URDF position
// and velocity limits and within +-10 rad/s^2, so the timed loop only reads memory. Every call is
// timed on its own with std::chrono::steady_clock after 1 000 warm-up calls; the same loop with
// nothing between the two clock reads is reported as the "clock" row, i.e. the timer's own cost,
// which is included in every other row and not subtracted.
#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "dynamics/dynamics_model.hpp"

namespace
{
using ur5e_teaching_controllers::dynamics::DynamicsModel;
using Clock = std::chrono::steady_clock;

const std::vector<std::string> kJoints{"shoulder_pan_joint", "shoulder_lift_joint",
                                       "elbow_joint",        "wrist_1_joint",
                                       "wrist_2_joint",      "wrist_3_joint"};
constexpr double kCycleUs = 2000.0;  // 500 Hz
constexpr int kWarmup = 1000;
constexpr double kMaxAcceleration = 10.0;

struct Options
{
  std::string urdf, output, commit{"unknown"}, build_type{"unknown"}, cpu{"unknown"};
  int calls{100000};
  unsigned seed{1};
};

struct Row
{
  std::string backend, function;
  std::vector<double> us;  // per-call wall time [us]
};

[[noreturn]] void usage(const char * msg)
{
  std::cerr << "bench_dynamics_timing: " << msg
            << "\nusage: bench_dynamics_timing --urdf FILE [--calls N] [--seed S] [--output CSV]"
               " [--commit SHA] [--build-type TYPE] [--cpu MODEL]\n";
  std::exit(2);
}

Options parse(int argc, char ** argv)
{
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (i + 1 >= argc) {
      usage(("missing value for " + a).c_str());
    }
    const std::string v = argv[++i];
    if (a == "--urdf") {
      o.urdf = v;
    } else if (a == "--calls") {
      o.calls = std::stoi(v);
    } else if (a == "--seed") {
      o.seed = static_cast<unsigned>(std::stoul(v));
    } else if (a == "--output") {
      o.output = v;
    } else if (a == "--commit") {
      o.commit = v;
    } else if (a == "--build-type") {
      o.build_type = v;
    } else if (a == "--cpu") {
      o.cpu = v;
    } else {
      usage(("unknown option " + a).c_str());
    }
  }
  if (o.urdf.empty()) {
    usage("--urdf is required");
  }
  if (o.calls < 1) {
    usage("--calls must be >= 1");
  }
  return o;
}

std::string read_file(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    usage(("cannot read " + path).c_str());
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

double us_between(Clock::time_point t0, Clock::time_point t1)
{
  return std::chrono::duration<double, std::micro>(t1 - t0).count();
}

/// Nearest-rank percentile of a sorted sample.
double percentile(const std::vector<double> & sorted, double p)
{
  const auto n = sorted.size();
  auto rank = static_cast<std::size_t>(p / 100.0 * static_cast<double>(n) + 0.999999);
  rank = std::clamp<std::size_t>(rank, 1, n);
  return sorted[rank - 1];
}

double mean(const std::vector<double> & v)
{
  double s = 0.0;
  for (const double x : v) {
    s += x;
  }
  return s / static_cast<double>(v.size());
}
}  // namespace

int main(int argc, char ** argv)
{
  const Options opt = parse(argc, argv);
  const std::string urdf = read_file(opt.urdf);
  const auto n_calls = static_cast<std::size_t>(opt.calls);

  std::vector<Row> rows;
  double sink = 0.0;  // consumed below so the timed calls cannot be optimised away

  {
    Row clock_row{"none", "clock", {}};
    clock_row.us.reserve(n_calls);
    for (std::size_t i = 0; i < n_calls; ++i) {
      const auto t0 = Clock::now();
      const auto t1 = Clock::now();
      clock_row.us.push_back(us_between(t0, t1));
    }
    rows.push_back(std::move(clock_row));
  }

  for (const std::string backend : {"pinocchio", "kdl"}) {
    std::unique_ptr<DynamicsModel> model =
      ur5e_teaching_controllers::dynamics::make_dynamics_model(backend, urdf, kJoints, "tool0");
    const auto n = static_cast<Eigen::Index>(model->dof());

    std::mt19937 rng(opt.seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    std::vector<Eigen::VectorXd> q(n_calls, Eigen::VectorXd(n)), v(q), a(q);
    for (std::size_t i = 0; i < n_calls; ++i) {
      for (Eigen::Index j = 0; j < n; ++j) {
        const double lo = model->lower_limits()[j];
        const double hi = model->upper_limits()[j];
        q[i][j] = lo + (hi - lo) * 0.5 * (unit(rng) + 1.0);
        v[i][j] = model->velocity_limits()[j] * unit(rng);
        a[i][j] = kMaxAcceleration * unit(rng);
      }
    }

    for (int i = 0; i < kWarmup; ++i) {
      const std::size_t k = static_cast<std::size_t>(i) % n_calls;
      sink += model->gravity(q[k])[0] + model->rnea(q[k], v[k], a[k])[0];
    }

    Row gravity_row{backend, "gravity", {}};
    gravity_row.us.reserve(n_calls);
    for (std::size_t i = 0; i < n_calls; ++i) {
      const auto t0 = Clock::now();
      const double x = model->gravity(q[i])[1];
      const auto t1 = Clock::now();
      sink += x;
      gravity_row.us.push_back(us_between(t0, t1));
    }
    rows.push_back(std::move(gravity_row));

    Row rnea_row{backend, "rnea", {}};
    rnea_row.us.reserve(n_calls);
    for (std::size_t i = 0; i < n_calls; ++i) {
      const auto t0 = Clock::now();
      const double x = model->rnea(q[i], v[i], a[i])[1];
      const auto t1 = Clock::now();
      sink += x;
      rnea_row.us.push_back(us_between(t0, t1));
    }
    rows.push_back(std::move(rnea_row));
  }

  std::ostringstream csv;
  csv << "bench,git_commit,build_type,cpu,seed,calls,backend,function,median_us,p95_us,mean_us,"
         "max_us,median_cycle_fraction,p95_cycle_fraction\n";
  std::printf(
    "%-10s %-8s %10s %10s %10s %10s %14s\n", "backend", "function", "median_us", "p95_us",
    "mean_us", "max_us", "p95 of 2 ms");
  for (auto & row : rows) {
    std::sort(row.us.begin(), row.us.end());
    const double med = percentile(row.us, 50.0);
    const double p95 = percentile(row.us, 95.0);
    const double avg = mean(row.us);
    const double mx = row.us.back();
    std::printf(
      "%-10s %-8s %10.3f %10.3f %10.3f %10.1f %13.4f%%\n", row.backend.c_str(),
      row.function.c_str(), med, p95, avg, mx, 100.0 * p95 / kCycleUs);
    char line[512];
    std::snprintf(
      line, sizeof(line),
      "dynamics_timing,%s,%s,\"%s\",%u,%d,%s,%s,%.4f,%.4f,%.4f,%.2f,%.6f,%.6f\n",
      opt.commit.c_str(), opt.build_type.c_str(), opt.cpu.c_str(), opt.seed, opt.calls,
      row.backend.c_str(), row.function.c_str(), med, p95, avg, mx, med / kCycleUs, p95 / kCycleUs);
    csv << line;
  }
  std::fprintf(stderr, "(checksum %g)\n", sink);

  if (!opt.output.empty()) {
    std::ofstream f(opt.output);
    if (!f) {
      std::cerr << "bench_dynamics_timing: cannot write " << opt.output << "\n";
      return 1;
    }
    f << csv.str();
  }
  return 0;
}
