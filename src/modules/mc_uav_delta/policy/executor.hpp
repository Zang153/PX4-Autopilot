#pragma once

#include <rl_tools/inference/executor/executor.h>
#include <rl_tools/inference/executor/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

#include "observation.hpp"

namespace mc_uav_delta
{

template <typename T_TYPE_POLICY, typename T_TI, typename T_TIMESTAMP, typename T_POLICY,
	  T_TIMESTAMP T_CONTROL_INTERVAL_INTERMEDIATE_NS, T_TIMESTAMP T_CONTROL_INTERVAL_NATIVE_NS,
	  bool T_FORCE_SYNC_INTERMEDIATE = false, T_TI T_FORCE_SYNC_NATIVE = 0,
	  bool T_FORCE_SYNC_NATIVE_RUNTIME = false,
	  typename T_WARNING_LEVELS = rl_tools::inference::executor::WarningLevelsDefault<T_TYPE_POLICY>,
	  bool T_DYNAMIC_ALLOCATION = false>
struct PolicyExecutorSpecification {
	using T = typename T_TYPE_POLICY::DEFAULT;
	using TI = T_TI;
	using TIMESTAMP = T_TIMESTAMP;
	static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
	using EXECUTOR_SPEC = rl_tools::inference::executor::Specification<T_TYPE_POLICY, TI, TIMESTAMP, T_POLICY,
	      T_CONTROL_INTERVAL_INTERMEDIATE_NS, T_CONTROL_INTERVAL_NATIVE_NS, T_FORCE_SYNC_INTERMEDIATE,
	      T_FORCE_SYNC_NATIVE, T_FORCE_SYNC_NATIVE_RUNTIME, T_WARNING_LEVELS, T_DYNAMIC_ALLOCATION>;
};

template <typename SPEC>
struct PolicyExecutor {
	using T = typename SPEC::T;
	using TI = typename SPEC::TI;
	static constexpr bool DYNAMIC_ALLOCATION = SPEC::DYNAMIC_ALLOCATION;
	static constexpr TI INPUT_DIM = POLICY_OBSERVATION_DIM;
	static constexpr TI OUTPUT_DIM = POLICY_ACTION_DIM;

	rl_tools::Tensor<rl_tools::tensor::Specification<T, TI, rl_tools::tensor::Shape<TI, 1, INPUT_DIM>, DYNAMIC_ALLOCATION>> input;
	rl_tools::Tensor<rl_tools::tensor::Specification<T, TI, rl_tools::tensor::Shape<TI, 1, OUTPUT_DIM>, DYNAMIC_ALLOCATION>> output;
	rl_tools::inference::Executor<typename SPEC::EXECUTOR_SPEC> executor;
	ActionHistory action_history;
};

template <typename DEVICE, typename SPEC, typename POLICY, typename RNG>
void reset(DEVICE &device, PolicyExecutor<SPEC> &executor, POLICY &policy, RNG &rng)
{
	executor.action_history.reset();
	rl_tools::reset(device, executor.executor, policy, rng);
}

template <typename DEVICE, typename SPEC>
void observe(DEVICE &device, PolicyExecutor<SPEC> &executor, const PolicyObservation &observation)
{
	float flattened[PolicyExecutor<SPEC>::INPUT_DIM];
	flattenObservation(observation, executor.action_history.mean(), flattened);

	for (typename DEVICE::index_t i = 0; i < PolicyExecutor<SPEC>::INPUT_DIM; ++i) {
		rl_tools::set(device, executor.input, flattened[i], 0, i);
	}
}

template <typename DEVICE, typename SPEC, typename POLICY, typename RNG>
auto control(DEVICE &device, PolicyExecutor<SPEC> &executor, typename SPEC::TIMESTAMP nanoseconds,
	     POLICY &policy, const PolicyObservation &observation, PolicyAction &action, RNG &rng)
{
	executor.action_history.update(observation.previous_action);
	observe(device, executor, observation);
	auto status = rl_tools::control(device, executor.executor, nanoseconds, policy, executor.input, executor.output, rng);

	for (typename DEVICE::index_t i = 0; i < PolicyExecutor<SPEC>::OUTPUT_DIM; ++i) {
		action.action[i] = rl_tools::get(device, executor.output, 0, i);
	}

	const bool native_step = status.source == decltype(status.source)::CONTROL
				 && status.step_type == decltype(status.step_type)::NATIVE;
	executor.action_history.complete(native_step);
	return status;
}

} // namespace mc_uav_delta
