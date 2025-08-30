/* Copyright 2022 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "all_ops_resolver.h"

// #include "tensorflow/lite/micro/kernels/micro_ops.h"
void AllOpsResolver(tflite::MicroMutableOpResolver<128> *resolver) 
{
  // Please keep this list of Builtin Operators in alphabetical order.
  resolver->AddAbs();
  resolver->AddAdd();
  resolver->AddAddN();
  resolver->AddArgMax();
  resolver->AddArgMin();
  resolver->AddAssignVariable();
  resolver->AddAveragePool2D();
  resolver->AddBatchToSpaceNd();
  resolver->AddBroadcastArgs();
  resolver->AddBroadcastTo();
  resolver->AddCallOnce();
  resolver->AddCast();
  resolver->AddCeil();
  resolver->AddCircularBuffer();
  resolver->AddConcatenation();
  resolver->AddConv2D();
  resolver->AddCos();
  resolver->AddCumSum();
  resolver->AddDepthToSpace();
  resolver->AddDepthwiseConv2D();
  resolver->AddDequantize();
  resolver->AddDetectionPostprocess();
  resolver->AddDiv();
  resolver->AddElu();
  resolver->AddEqual();
  resolver->AddEthosU();
  resolver->AddExp();
  resolver->AddExpandDims();
  resolver->AddFill();
  resolver->AddFloor();
  resolver->AddFloorDiv();
  resolver->AddFloorMod();
  resolver->AddFullyConnected();
  resolver->AddGather();
  resolver->AddGatherNd();
  resolver->AddGreater();
  resolver->AddGreaterEqual();
  resolver->AddHardSwish();
  resolver->AddIf();
  resolver->AddL2Normalization();
  resolver->AddL2Pool2D();
  resolver->AddLeakyRelu();
  resolver->AddLess();
  resolver->AddLessEqual();
  resolver->AddLog();
  resolver->AddLogicalAnd();
  resolver->AddLogicalNot();
  resolver->AddLogicalOr();
  resolver->AddLogistic();
  resolver->AddLogSoftmax();
  resolver->AddMaxPool2D();
  resolver->AddMaximum();
  resolver->AddMean();
  resolver->AddMinimum();
  resolver->AddMirrorPad();
  resolver->AddMul();
  resolver->AddNeg();
  resolver->AddNotEqual();
  resolver->AddPack();
  resolver->AddPad();
  resolver->AddPadV2();
  resolver->AddPrelu();
  resolver->AddQuantize();
  resolver->AddReadVariable();
  resolver->AddReduceMax();
  resolver->AddRelu();
  resolver->AddRelu6();
  resolver->AddReshape();
  resolver->AddResizeBilinear();
  resolver->AddResizeNearestNeighbor();
  resolver->AddRound();
  resolver->AddRsqrt();
  resolver->AddSelectV2();
  resolver->AddShape();
  resolver->AddSin();
  resolver->AddSlice();
  resolver->AddSoftmax();
  resolver->AddSpaceToBatchNd();
  resolver->AddSpaceToDepth();
  resolver->AddSplit();
  resolver->AddSplitV();
  resolver->AddSqrt();
  resolver->AddSquare();
  resolver->AddSquaredDifference();
  resolver->AddSqueeze();
  resolver->AddStridedSlice();
  resolver->AddSub();
  resolver->AddSum();
  resolver->AddSvdf();
  resolver->AddTanh();
  resolver->AddTranspose();
  resolver->AddTransposeConv();
  resolver->AddUnidirectionalSequenceLSTM();
  resolver->AddUnpack();
  resolver->AddVarHandle();
  resolver->AddWhile();
  resolver->AddZerosLike();
}

