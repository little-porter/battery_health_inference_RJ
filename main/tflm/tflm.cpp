#include "tflm.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "all_ops_resolver.h"
// #include "model_data1.c"

#define TFLM_AREA_SIZE  (3*1024*1024)           // 3M   Tensor param region (张量区域)


// model op resolver(method)
static  tflite::MicroMutableOpResolver<128>  micro_op_resolver;
// static tflite::AllOpsResolver micro_op_resolver;

extern const unsigned char model_data1[];
extern float  test_data[280][3];
extern const float test_Time[280];
extern const float test_v[280];
extern const float test_step[280];

static uint8_t *tflm_area = NULL;               //寮犻噺鍖哄煙

tflm_module_t tflm_soc;


// static void AllOpsResolver(tflite::MicroMutableOpResolver<128> *resolver) 
// {
//     // Please keep this list of Builtin Operators in alphabetical order.
//     resolver->AddAbs();
//     resolver->AddAdd();
//     resolver->AddAddN();
//     resolver->AddArgMax();
//     resolver->AddArgMin();
//     resolver->AddAssignVariable();
//     resolver->AddAveragePool2D();
//     resolver->AddBatchToSpaceNd();
//     resolver->AddBroadcastArgs();
//     resolver->AddBroadcastTo();
//     resolver->AddCallOnce();
//     resolver->AddCast();
//     resolver->AddCeil();
//     resolver->AddCircularBuffer();
//     resolver->AddConcatenation();
//     resolver->AddConv2D();
//     resolver->AddCos();
//     resolver->AddCumSum();
//     resolver->AddDepthToSpace();
//     resolver->AddDepthwiseConv2D();
//     resolver->AddDequantize();
//     resolver->AddDetectionPostprocess();
//     resolver->AddDiv();
//     resolver->AddElu();
//     resolver->AddEqual();
//     resolver->AddEthosU();
//     resolver->AddExp();
//     resolver->AddExpandDims();
//     resolver->AddFill();
//     resolver->AddFloor();
//     resolver->AddFloorDiv();
//     resolver->AddFloorMod();
//     resolver->AddFullyConnected();
//     resolver->AddGather();
//     resolver->AddGatherNd();
//     resolver->AddGreater();
//     resolver->AddGreaterEqual();
//     resolver->AddHardSwish();
//     resolver->AddIf();
//     resolver->AddL2Normalization();
//     resolver->AddL2Pool2D();
//     resolver->AddLeakyRelu();
//     resolver->AddLess();
//     resolver->AddLessEqual();
//     resolver->AddLog();
//     resolver->AddLogicalAnd();
//     resolver->AddLogicalNot();
//     resolver->AddLogicalOr();
//     resolver->AddLogistic();
//     resolver->AddLogSoftmax();
//     resolver->AddMaxPool2D();
//     resolver->AddMaximum();
//     resolver->AddMean();
//     resolver->AddMinimum();
//     resolver->AddMirrorPad();
//     resolver->AddMul();
//     resolver->AddNeg();
//     resolver->AddNotEqual();
//     resolver->AddPack();
//     resolver->AddPad();
//     resolver->AddPadV2();
//     resolver->AddPrelu();
//     resolver->AddQuantize();
//     resolver->AddReadVariable();
//     resolver->AddReduceMax();
//     resolver->AddRelu();
//     resolver->AddRelu6();
//     resolver->AddReshape();
//     resolver->AddResizeBilinear();
//     resolver->AddResizeNearestNeighbor();
//     resolver->AddRound();
//     resolver->AddRsqrt();
//     resolver->AddSelectV2();
//     resolver->AddShape();
//     resolver->AddSin();
//     resolver->AddSlice();
//     resolver->AddSoftmax();
//     resolver->AddSpaceToBatchNd();
//     resolver->AddSpaceToDepth();
//     resolver->AddSplit();
//     resolver->AddSplitV();
//     resolver->AddSqrt();
//     resolver->AddSquare();
//     resolver->AddSquaredDifference();
//     resolver->AddSqueeze();
//     resolver->AddStridedSlice();
//     resolver->AddSub();
//     resolver->AddSum();
//     resolver->AddSvdf();
//     resolver->AddTanh();
//     resolver->AddTranspose();
//     resolver->AddTransposeConv();
//     resolver->AddUnidirectionalSequenceLSTM();
//     resolver->AddUnpack();
//     resolver->AddVarHandle();
//     resolver->AddWhile();
//     resolver->AddZerosLike();
// }

/*鍒濓拷?锟藉寲瑙ｉ噴锟??*/
extern "C"  void tflm_create(tflm_module_t *tflm)
{
    //鏁版嵁绌洪棿鍒ゆ柇
    if (tflm_area == NULL || tflm->model_data == NULL) 
    {
        printf("tflm_area don't allocate memory or model_data is null!");
        return;
    }
     //鍔犺浇妯″瀷
    const tflite::Model *tl_model = tflite::GetModel(tflm->model_data);
    if (tl_model->version() != TFLITE_SCHEMA_VERSION) 
    {
        MicroPrintf("Model provided is schema version %d not equal to supported "
    		"version %d.", tl_model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }
    // static tflite::MicroInterpreter interpreter_temp(tl_model, micro_op_resolver, tf_area, TFLM_AREA_SIZE,nullptr,nullptr,true);
    /* 鍒涘缓瑙ｉ噴锟?? */
    tflite::MicroInterpreter *interpreter = new tflite::MicroInterpreter(tl_model, micro_op_resolver, tflm_area, TFLM_AREA_SIZE);
    /*鍒嗛厤寮犻噺鍐呭瓨*/
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    MicroPrintf("interpreter is regester !");
    if (allocate_status != kTfLiteOk) {
        MicroPrintf("AllocateTensors() failed");
        return;
    }
    // if(interpreter->input_tensor(0)->dims->size != 3)
    // {
    //     MicroPrintf("妯″瀷閿欙拷??!");
    //     interpreter->~MicroInterpreter();       //閲婃斁璧勬簮
    //     return;
    // }

    tflm->interpreter = interpreter;

    tflm->input_row = interpreter->input_tensor(0)->dims->data[1];
    tflm->input_col = interpreter->input_tensor(0)->dims->data[2];
    

    for(int i = 0; i < interpreter->input_tensor(0)->dims->size; i++)
    {
        MicroPrintf("input_tensor_dims[%d] = %d", i, interpreter->input_tensor(0)->dims->data[i]);
    }
    for(int i = 0; i < interpreter->output_tensor(0)->dims->size; i++)
    {
        if(i == 0)   tflm->result_num = interpreter->output_tensor(0)->dims->data[i];
        else         tflm->result_num *= interpreter->output_tensor(0)->dims->data[i];
        MicroPrintf("output_tensor_dims[%d] = %d", i, interpreter->output_tensor(0)->dims->data[i]);
    }

    // ESP_LOGI(TAG, "Initializing tflm is over!");
}

extern "C" void tflm_init(void)
{
    /*鍒濓拷?锟藉寲寮犻噺鍐呭瓨*/
    tflm_area = (uint8_t *)heap_caps_malloc(TFLM_AREA_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    // for(int i = 0;i < 280; i++)
    // {
    //     test_data[i][0] = test_step[i];
    //     test_data[i][1] = test_v[i];
    //     test_data[i][2] = test_Time[i];
    // }

    /*鍒濓拷?锟藉寲妯″瀷浣跨敤鏂规硶*/
    AllOpsResolver(&micro_op_resolver);

    /*鍒濓拷?锟藉寲SOC妯″瀷*/
    // tflm_soc.model_data = model_data1;
    // tflm_create(&tflm_soc);
#ifdef USE_SOH_MODEL 
    /*鍒濓拷?锟藉寲SOH妯″瀷*/
    tflm_soh.model_data = model_data1;
    tflm_soh.tf_area = tflm_area;
    tflm_interpreter_init(tflm_soh.interpreter, tflm_soh.model_data, tflm_soh.tf_area);
#endif
}


extern "C" void tflm_run(tflm_module_t *tflm,float *input_data,uint32_t input_num,float *output_data,uint32_t output_num)
{
    // tflm_interpreter_init(&tflm_soc.interpreter, tflm_soc.model_data, tflm_soc.tf_area);
   if(tflm->interpreter == NULL)
   {
      MicroPrintf("interpreter is null!");
   }

    tflite::MicroInterpreter *interpreter = (tflite::MicroInterpreter *)tflm->interpreter;
    interpreter->AllocateTensors();


    TfLiteTensor *input  = interpreter->input(0);
    TfLiteTensor *output = interpreter->output(0);
    

    /*璁剧疆杈撳叆寮犻噺鏁版嵁*/
    for(int i = 0; i < input_num; i++)
    {
        input->data.f[i] = input_data[i];
    }

    /*杩涳拷?锟芥帹锟??*/
    TfLiteStatus invoke_status = interpreter->Invoke();
    if(invoke_status != kTfLiteOk) 
    {
        MicroPrintf("Invoke failed\n");
        return;
    }

    if(output->type != kTfLiteFloat32)  return;

    int num = output->bytes/4;
    if(num > output_num)    num = output_num;


    /*鑾峰彇杈撳嚭寮犻噺鏁版嵁*/
    for (int i = 0; i < num; i++) 
    {
        if(output->data.f[i] > 1)
        {
            output_data[i] = 1;
        }
        else if(output->data.f[i] < 0)
        {
            output_data[i] = 0;
        }
        else
        {
            output_data[i] = output->data.f[i];
        }
    }
}

extern "C" void tflm_release(tflm_module_t *tflm)
{
    if(tflm->interpreter == nullptr || tflm == nullptr)    return;
    tflm->model_data = NULL;
    tflm->input_col = 0;
    tflm->input_row = 0;
    tflite::MicroInterpreter *interpreter = (tflite::MicroInterpreter *)tflm->interpreter;

    interpreter->~MicroInterpreter();
}




