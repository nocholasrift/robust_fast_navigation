#ifndef ROBUST_FAST_NAVIGATION_IMPORTANCE_SAMPLER_H
#define ROBUST_FAST_NAVIGATION_IMPORTANCE_SAMPLER_H

#include <Eigen/Core>
#include <chrono>
#include <iostream>
#include <random>
#include <stdexcept>

// templated importance sampling
template <typename CostFn, typename Numeric> class ImportanceSampler {
public:
  using Matrix = Eigen::Matrix<Numeric, Eigen::Dynamic, Eigen::Dynamic>;
  using Vector = Eigen::Matrix<Numeric, Eigen::Dynamic, 1>;

  ImportanceSampler(CostFn cost_fn, Vector std_dev, Numeric temperature,
                    unsigned int num_iterations, unsigned int num_samples);

  ImportanceSampler(CostFn cost_fn, Numeric std_dev, Numeric temperature,
                    unsigned int num_iterations, unsigned int num_samples);

  ~ImportanceSampler() = default;

  Matrix optimize(const Matrix &initial_state);

private:
  std::random_device rd_;
  std::mt19937 gen_{rd_()};
  std::normal_distribution<Numeric> unit_normal_distr_;

  Vector std_dev_{Vector::Constant(1, -1)};

  Vector costs_;
  Vector weights_;
  Vector sample_;

  CostFn cost_fn_;

  Numeric temperature_{1.0};

  unsigned int num_samples_{1000};
  unsigned int num_iterations_{10};

  static constexpr Numeric kZeroMean = 0.0;
  static constexpr Numeric kUnitStd = 1.0;
};

// implementation
template <typename CostFn, typename Numeric>
ImportanceSampler<CostFn, Numeric>::ImportanceSampler(
    CostFn cost_fn, Vector std_dev, Numeric temperature,
    unsigned int num_iterations, unsigned int num_samples)
    : cost_fn_(cost_fn), std_dev_(std_dev),
      unit_normal_distr_(kZeroMean, kUnitStd), temperature_(temperature),
      num_iterations_(num_iterations), num_samples_(num_samples),
      costs_(num_samples), weights_(num_samples) {}

// don't know the dimensions yet, need to wait before
// initializing std_dev
template <typename CostFn, typename Numeric>
ImportanceSampler<CostFn, Numeric>::ImportanceSampler(
    CostFn cost_fn, Numeric std_dev, Numeric temperature,
    unsigned int num_iterations, unsigned int num_samples)
    : cost_fn_(cost_fn), std_dev_(Vector::Constant(1, std_dev)),
      unit_normal_distr_(kZeroMean, kUnitStd), temperature_(temperature),
      num_iterations_(num_iterations), num_samples_(num_samples),
      costs_(num_samples), weights_(num_samples) {}

template <typename CostFn, typename Numeric>
typename ImportanceSampler<CostFn, Numeric>::Matrix
ImportanceSampler<CostFn, Numeric>::optimize(const Matrix &initial_state) {
  auto start = std::chrono::high_resolution_clock::now();
  const Numeric kInf = 1e6;

  // flatten state
  Vector state_vec = Vector::Map(initial_state.data(), initial_state.size());
  sample_.resize(state_vec.size());

  // populate std_dev vec to match size of state vec
  if (std_dev_.size() != state_vec.size()) {
    std_dev_ = Vector::Constant(state_vec.size(), std_dev_[0]);
  }

  Numeric temp_inverse = 1.0 / temperature_;

  Matrix perturbations(num_samples_, state_vec.size());
  for (unsigned int iter = 0; iter < num_iterations_; ++iter) {
    Numeric min_cost = kInf;

    // construct all perturbations at start
    perturbations =
        Matrix::NullaryExpr(num_samples_, state_vec.size(),
                            [this]() { return unit_normal_distr_(gen_); });

    perturbations = perturbations * std_dev_.asDiagonal();

    // get costs per perturbation
    for (unsigned int ind_sample = 0; ind_sample < num_samples_; ++ind_sample) {

      // perturb state vector then evaluate its cost
      // tell eigen there is no alias (a = a + b), so
      // no need to create intermediate temp objects

      // state_vec is Nx1 and pert.row is 1xN.
      sample_.noalias() = state_vec + perturbations.row(ind_sample).transpose();
      costs_[ind_sample] = cost_fn_(sample_);
      if (costs_[ind_sample] < min_cost) {
        min_cost = costs_[ind_sample];
      }
    }

    // compute trajectory weights
    weights_ = (-temp_inverse * (costs_.array() - min_cost)).exp();
    weights_ /= weights_.sum();

    // update state vector
    state_vec += perturbations.transpose() * weights_;
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Execution time: " << duration.count() << " milliseconds"
            << std::endl;

  return Eigen::Map<Matrix>(state_vec.data(), initial_state.rows(),
                            initial_state.cols());
}

#endif
