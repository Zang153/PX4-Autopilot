#pragma once

#include <rl_tools/containers/tensor/tensor.h>
#include <rl_tools/numeric_types/policy.h>
#include <rl_tools/nn/parameters/parameters.h>
#include <rl_tools/nn/layers/dense/layer.h>
#include <rl_tools/nn/layers/gru/layer.h>
#include <rl_tools/nn_models/sequential/model.h>

#include "observation.hpp"

namespace mc_uav_delta::policy
{

namespace layer_0
{
using TYPE_POLICY = rl_tools::numeric_types::Policy<float>;
using CONFIG = rl_tools::nn::layers::dense::Configuration<TYPE_POLICY, unsigned long, 32,
      rl_tools::nn::activation_functions::ActivationFunction::RELU,
      rl_tools::nn::layers::dense::DefaultInitializer<TYPE_POLICY, unsigned long>,
      rl_tools::nn::parameters::groups::Input>;
using TEMPLATE = rl_tools::nn::layers::dense::BindConfiguration<CONFIG>;
using INPUT_SHAPE = rl_tools::tensor::Shape<unsigned long, 500, 2, POLICY_OBSERVATION_DIM>;
} // namespace layer_0

namespace layer_1
{
using TYPE_POLICY = rl_tools::numeric_types::Policy<float>;
using CONFIG = rl_tools::nn::layers::gru::Configuration<TYPE_POLICY, unsigned long, 32,
      rl_tools::nn::parameters::groups::Normal, false>;
using TEMPLATE = rl_tools::nn::layers::gru::BindConfiguration<CONFIG>;
using INPUT_SHAPE = rl_tools::tensor::Shape<unsigned long, 500, 2, 32>;
} // namespace layer_1

namespace layer_2
{
using TYPE_POLICY = rl_tools::numeric_types::Policy<float>;
using CONFIG = rl_tools::nn::layers::dense::Configuration<TYPE_POLICY, unsigned long, POLICY_ACTION_DIM,
      rl_tools::nn::activation_functions::ActivationFunction::TANH,
      rl_tools::nn::layers::dense::DefaultInitializer<TYPE_POLICY, unsigned long>,
      rl_tools::nn::parameters::groups::Output>;
using TEMPLATE = rl_tools::nn::layers::dense::BindConfiguration<CONFIG>;
using INPUT_SHAPE = rl_tools::tensor::Shape<unsigned long, 500, 2, 32>;
} // namespace layer_2

namespace model_definition
{
using CAPABILITY = rl_tools::nn::capability::Forward<true, true>;
template <typename T_CONTENT, typename T_NEXT_MODULE = rl_tools::nn_models::sequential::OutputModule>
using Module = typename rl_tools::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
using MODULE_CHAIN = Module<layer_0::TEMPLATE, Module<layer_1::TEMPLATE, Module<layer_2::TEMPLATE>>>;
using MODEL = typename rl_tools::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, layer_0::INPUT_SHAPE>;
} // namespace model_definition

using TYPE = model_definition::MODEL;
static_assert(TYPE::INPUT_SHAPE::LAST == POLICY_OBSERVATION_DIM);
static_assert(TYPE::OUTPUT_SHAPE::LAST == POLICY_ACTION_DIM);
static_assert(layer_2::CONFIG::ACTIVATION_FUNCTION == rl_tools::nn::activation_functions::ActivationFunction::TANH);

namespace example
{
namespace input
{
using SHAPE = rl_tools::tensor::Shape<unsigned long, 500, 2, POLICY_OBSERVATION_DIM>;
using SPEC = rl_tools::tensor::Specification<float, unsigned long, SHAPE, true,
      rl_tools::tensor::RowMajorStride<SHAPE>, true>;
} // namespace input

namespace output
{
using SHAPE = rl_tools::tensor::Shape<unsigned long, 500, 2, POLICY_ACTION_DIM>;
using SPEC = rl_tools::tensor::Specification<float, unsigned long, SHAPE, true,
      rl_tools::tensor::RowMajorStride<SHAPE>, true>;
} // namespace output
} // namespace example

} // namespace mc_uav_delta::policy
