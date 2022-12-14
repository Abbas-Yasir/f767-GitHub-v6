/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Thu Nov  3 14:16:11 2022
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */


#include "network.h"

#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "layers.h"

#undef AI_TOOLS_VERSION_MAJOR
#undef AI_TOOLS_VERSION_MINOR
#undef AI_TOOLS_VERSION_MICRO
#define AI_TOOLS_VERSION_MAJOR 5
#define AI_TOOLS_VERSION_MINOR 1
#define AI_TOOLS_VERSION_MICRO 2


#undef AI_TOOLS_API_VERSION_MAJOR
#undef AI_TOOLS_API_VERSION_MINOR
#undef AI_TOOLS_API_VERSION_MICRO
#define AI_TOOLS_API_VERSION_MAJOR 1
#define AI_TOOLS_API_VERSION_MINOR 3
#define AI_TOOLS_API_VERSION_MICRO 0

#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "1158d014aaf003027cb369a7675773e2"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     "(rev-5.1.2)"
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "Thu Nov  3 14:16:11 2022"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

/**  Forward network declaration section  *************************************/
AI_STATIC ai_network AI_NET_OBJ_INSTANCE;


/**  Forward network array declarations  **************************************/
AI_STATIC ai_array conv2d_1_scratch2_array;   /* Array #0 */
AI_STATIC ai_array conv2d_1_scratch1_array;   /* Array #1 */
AI_STATIC ai_array conv2d_1_scratch0_array;   /* Array #2 */
AI_STATIC ai_array dense_3_bias_array;   /* Array #3 */
AI_STATIC ai_array dense_3_weights_array;   /* Array #4 */
AI_STATIC ai_array conv2d_1_bias_array;   /* Array #5 */
AI_STATIC ai_array conv2d_1_weights_array;   /* Array #6 */
AI_STATIC ai_array serving_default_input_20_output_array;   /* Array #7 */
AI_STATIC ai_array conversion_0_output_array;   /* Array #8 */
AI_STATIC ai_array conv2d_1_output_array;   /* Array #9 */
AI_STATIC ai_array dense_3_output_array;   /* Array #10 */
AI_STATIC ai_array dense_3_fmt_output_array;   /* Array #11 */
AI_STATIC ai_array nl_4_output_array;   /* Array #12 */
AI_STATIC ai_array nl_4_fmt_output_array;   /* Array #13 */


/**  Forward network tensor declarations  *************************************/
AI_STATIC ai_tensor conv2d_1_scratch2;   /* Tensor #0 */
AI_STATIC ai_tensor conv2d_1_scratch1;   /* Tensor #1 */
AI_STATIC ai_tensor conv2d_1_scratch0;   /* Tensor #2 */
AI_STATIC ai_tensor dense_3_bias;   /* Tensor #3 */
AI_STATIC ai_tensor dense_3_weights;   /* Tensor #4 */
AI_STATIC ai_tensor conv2d_1_bias;   /* Tensor #5 */
AI_STATIC ai_tensor conv2d_1_weights;   /* Tensor #6 */
AI_STATIC ai_tensor serving_default_input_20_output;   /* Tensor #7 */
AI_STATIC ai_tensor conversion_0_output;   /* Tensor #8 */
AI_STATIC ai_tensor conv2d_1_output;   /* Tensor #9 */
AI_STATIC ai_tensor dense_3_output;   /* Tensor #10 */
AI_STATIC ai_tensor dense_3_fmt_output;   /* Tensor #11 */
AI_STATIC ai_tensor nl_4_output;   /* Tensor #12 */
AI_STATIC ai_tensor nl_4_fmt_output;   /* Tensor #13 */


/**  Forward network tensor chain declarations  *******************************/
AI_STATIC_CONST ai_tensor_chain conversion_0_chain;   /* Chain #0 */
AI_STATIC_CONST ai_tensor_chain conv2d_1_chain;   /* Chain #1 */
AI_STATIC_CONST ai_tensor_chain dense_3_chain;   /* Chain #2 */
AI_STATIC_CONST ai_tensor_chain dense_3_fmt_chain;   /* Chain #3 */
AI_STATIC_CONST ai_tensor_chain nl_4_chain;   /* Chain #4 */
AI_STATIC_CONST ai_tensor_chain nl_4_fmt_chain;   /* Chain #5 */


/**  Forward network layer declarations  **************************************/
AI_STATIC ai_layer_nl conversion_0_layer; /* Layer #0 */
AI_STATIC ai_layer_conv2d_nl_pool conv2d_1_layer; /* Layer #1 */
AI_STATIC ai_layer_dense dense_3_layer; /* Layer #2 */
AI_STATIC ai_layer_nl dense_3_fmt_layer; /* Layer #3 */
AI_STATIC ai_layer_nl nl_4_layer; /* Layer #4 */
AI_STATIC ai_layer_nl nl_4_fmt_layer; /* Layer #5 */


/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_scratch2_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 38400, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_scratch1_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 38400, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_scratch0_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 148, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 2, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 16, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 8, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 72, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  serving_default_input_20_output_array, AI_ARRAY_FORMAT_U8|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 19200, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  conversion_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 19200, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 8, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 2, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  dense_3_fmt_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  nl_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  nl_4_fmt_output_array, AI_ARRAY_FORMAT_S8|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 2, AI_STATIC)

/**  Array metadata declarations section  *************************************/
/* Int quant #0 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(conv2d_1_scratch2_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0235294122248888f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #1 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(conv2d_1_scratch1_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0235294122248888f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #2 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(dense_3_bias_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.00019844055350404233f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #3 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(dense_3_weights_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.011022712104022503f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #4 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(conv2d_1_bias_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 8,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(4.046152389491908e-05f, 7.6171595537744e-06f, 1.0696729077608325e-05f, 1.2882809642178472e-05f, 4.1947929275920615e-05f, 3.34818396368064e-05f, 1.4330941667139996e-05f, 1.6473653886350803e-05f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #5 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(conv2d_1_weights_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 8,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.010317687876522541f, 0.0019423755584284663f, 0.0027276657056063414f, 0.003285116283223033f, 0.010696721263229847f, 0.008537868969142437f, 0.003654389875009656f, 0.00420078169554472f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #6 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(serving_default_input_20_output_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_U8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.003921568859368563f),
    AI_PACK_UINTQ_ZP(0)))

/* Int quant #7 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(conversion_0_output_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.003921568859368563f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #8 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(conv2d_1_output_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.01800287887454033f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #9 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(dense_3_output_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.06302019208669662f),
    AI_PACK_INTQ_ZP(9)))

/* Int quant #10 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(nl_4_fmt_output_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.00390625f),
    AI_PACK_INTQ_ZP(-128)))

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_1_scratch2, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 80, 60), AI_STRIDE_INIT(4, 1, 1, 8, 640),
  1, &conv2d_1_scratch2_array, &conv2d_1_scratch2_intq)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_1_scratch1, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 80, 60), AI_STRIDE_INIT(4, 1, 1, 8, 640),
  1, &conv2d_1_scratch1_array, &conv2d_1_scratch1_intq)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_1_scratch0, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 148, 1, 1), AI_STRIDE_INIT(4, 1, 1, 148, 148),
  1, &conv2d_1_scratch0_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &dense_3_bias_array, &dense_3_bias_intq)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 8, 2, 1, 1), AI_STRIDE_INIT(4, 1, 8, 16, 16),
  1, &dense_3_weights_array, &dense_3_weights_intq)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_1_bias, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &conv2d_1_bias_array, &conv2d_1_bias_intq)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_1_weights, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 3, 8), AI_STRIDE_INIT(4, 1, 1, 3, 9),
  1, &conv2d_1_weights_array, &conv2d_1_weights_intq)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  serving_default_input_20_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 160, 120), AI_STRIDE_INIT(4, 1, 1, 1, 160),
  1, &serving_default_input_20_output_array, &serving_default_input_20_output_intq)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  conversion_0_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 160, 120), AI_STRIDE_INIT(4, 1, 1, 1, 160),
  1, &conversion_0_output_array, &conversion_0_output_intq)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  conv2d_1_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 1, 1, 8, 8),
  1, &conv2d_1_output_array, &conv2d_1_output_intq)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 1, 1, 2, 2),
  1, &dense_3_output_array, &dense_3_output_intq)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  dense_3_fmt_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &dense_3_fmt_output_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  nl_4_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &nl_4_output_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  nl_4_fmt_output, AI_STATIC,
  0x0, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 1, 1, 2, 2),
  1, &nl_4_fmt_output_array, &nl_4_fmt_output_intq)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  conversion_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &serving_default_input_20_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conversion_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conversion_0_layer, 0,
  NL_TYPE,
  nl, node_convert_integer,
  &AI_NET_OBJ_INSTANCE, &conv2d_1_layer, AI_STATIC,
  .tensors = &conversion_0_chain, 
)


AI_STATIC_CONST ai_i8 conv2d_1_nl_params_data[] = { -128, 127 };
AI_ARRAY_OBJ_DECLARE(
    conv2d_1_nl_params, AI_ARRAY_FORMAT_S8,
    conv2d_1_nl_params_data, conv2d_1_nl_params_data, 2, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conversion_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_1_weights, &conv2d_1_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv2d_1_scratch0, &conv2d_1_scratch1, &conv2d_1_scratch2)
)

AI_LAYER_OBJ_DECLARE(
  conv2d_1_layer, 1,
  OPTIMIZED_CONV2D_TYPE,
  conv2d_nl_pool, forward_conv2d_nl_pool_integer_SSSA_ch,
  &AI_NET_OBJ_INSTANCE, &dense_3_layer, AI_STATIC,
  .tensors = &conv2d_1_chain, 
  .groups = 1, 
  .nl_params = &conv2d_1_nl_params, 
  .nl_func = nl_func_clip_array_integer, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 2, 2), 
  .pool_size = AI_SHAPE_2D_INIT(80, 60), 
  .pool_stride = AI_SHAPE_2D_INIT(80, 60), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .pool_func = pool_func_ap_array_integer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv2d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_3_weights, &dense_3_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_3_layer, 3,
  DENSE_TYPE,
  dense, forward_dense_integer_SSSA,
  &AI_NET_OBJ_INSTANCE, &dense_3_fmt_layer, AI_STATIC,
  .tensors = &dense_3_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_3_fmt_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_fmt_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_3_fmt_layer, 3,
  NL_TYPE,
  nl, node_convert,
  &AI_NET_OBJ_INSTANCE, &nl_4_layer, AI_STATIC,
  .tensors = &dense_3_fmt_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_3_fmt_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_4_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_4_layer, 4,
  NL_TYPE,
  nl, forward_sm,
  &AI_NET_OBJ_INSTANCE, &nl_4_fmt_layer, AI_STATIC,
  .tensors = &nl_4_chain, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  nl_4_fmt_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &nl_4_fmt_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  nl_4_fmt_layer, 4,
  NL_TYPE,
  nl, node_convert,
  &AI_NET_OBJ_INSTANCE, &nl_4_fmt_layer, AI_STATIC,
  .tensors = &nl_4_fmt_chain, 
)


AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 128, 1,
                     NULL),
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 57756, 1,
                     NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &serving_default_input_20_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &nl_4_fmt_output),
  &conversion_0_layer, 0, NULL)



AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_buffer* activation_buffer)
{
  AI_ASSERT(net_ctx &&  activation_buffer && activation_buffer->data)

  ai_ptr activations = AI_PTR(AI_PTR_ALIGN(activation_buffer->data, 4));
  AI_ASSERT(activations)
  AI_UNUSED(net_ctx)

  {
    /* Updating activations (byte) offsets */
    conv2d_1_scratch2_array.data = AI_PTR(activations + 19348);
    conv2d_1_scratch2_array.data_start = AI_PTR(activations + 19348);
    conv2d_1_scratch1_array.data = AI_PTR(activations + 19348);
    conv2d_1_scratch1_array.data_start = AI_PTR(activations + 19348);
    conv2d_1_scratch0_array.data = AI_PTR(activations + 19200);
    conv2d_1_scratch0_array.data_start = AI_PTR(activations + 19200);
    serving_default_input_20_output_array.data = AI_PTR(NULL);
    serving_default_input_20_output_array.data_start = AI_PTR(NULL);
    conversion_0_output_array.data = AI_PTR(activations + 0);
    conversion_0_output_array.data_start = AI_PTR(activations + 0);
    conv2d_1_output_array.data = AI_PTR(activations + 57748);
    conv2d_1_output_array.data_start = AI_PTR(activations + 57748);
    dense_3_output_array.data = AI_PTR(activations + 0);
    dense_3_output_array.data_start = AI_PTR(activations + 0);
    dense_3_fmt_output_array.data = AI_PTR(activations + 4);
    dense_3_fmt_output_array.data_start = AI_PTR(activations + 4);
    nl_4_output_array.data = AI_PTR(activations + 12);
    nl_4_output_array.data_start = AI_PTR(activations + 12);
    nl_4_fmt_output_array.data = AI_PTR(NULL);
    nl_4_fmt_output_array.data_start = AI_PTR(NULL);
    
  }
  return true;
}



AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_buffer* weights_buffer)
{
  AI_ASSERT(net_ctx &&  weights_buffer && weights_buffer->data)

  ai_ptr weights = AI_PTR(weights_buffer->data);
  AI_ASSERT(weights)
  AI_UNUSED(net_ctx)

  {
    /* Updating weights (byte) offsets */
    
    dense_3_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_3_bias_array.data = AI_PTR(weights + 120);
    dense_3_bias_array.data_start = AI_PTR(weights + 120);
    dense_3_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_3_weights_array.data = AI_PTR(weights + 104);
    dense_3_weights_array.data_start = AI_PTR(weights + 104);
    conv2d_1_bias_array.format |= AI_FMT_FLAG_CONST;
    conv2d_1_bias_array.data = AI_PTR(weights + 72);
    conv2d_1_bias_array.data_start = AI_PTR(weights + 72);
    conv2d_1_weights_array.format |= AI_FMT_FLAG_CONST;
    conv2d_1_weights_array.data = AI_PTR(weights + 0);
    conv2d_1_weights_array.data_start = AI_PTR(weights + 0);
  }

  return true;
}


/**  PUBLIC APIs SECTION  *****************************************************/

AI_API_ENTRY
ai_bool ai_network_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if ( report && net_ctx )
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = {AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR,
                            AI_TOOLS_API_VERSION_MICRO, 0x0},

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 499262,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .activations       = AI_STRUCT_INIT,
      .params            = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x0,
    };

    if ( !ai_platform_api_get_network_report(network, &r) ) return false;

    *report = r;
    return true;
  }

  return false;
}

AI_API_ENTRY
ai_error ai_network_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}

AI_API_ENTRY
ai_error ai_network_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    &AI_NET_OBJ_INSTANCE,
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}

AI_API_ENTRY
ai_handle ai_network_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}

AI_API_ENTRY
ai_bool ai_network_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = ai_platform_network_init(network, params);
  if ( !net_ctx ) return false;

  ai_bool ok = true;
  ok &= network_configure_weights(net_ctx, &params->params);
  ok &= network_configure_activations(net_ctx, &params->activations);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}

AI_API_ENTRY
ai_i32 ai_network_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}


#undef AI_NETWORK_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_VERSION_MAJOR
#undef AI_TOOLS_VERSION_MINOR
#undef AI_TOOLS_VERSION_MICRO
#undef AI_TOOLS_API_VERSION_MAJOR
#undef AI_TOOLS_API_VERSION_MINOR
#undef AI_TOOLS_API_VERSION_MICRO
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

