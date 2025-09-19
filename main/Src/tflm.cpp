#include "tflm.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
// #include "all_ops_resolver.h"
// #include "model_data1.c"

#define TFLM_AREA_SIZE  (300*1024)           // 3M   张量数据存储空间（存储模型输入、输出、中间�?�算结果�??


// 2. 创建操作符解析器(模型使用方法)
static  tflite::MicroMutableOpResolver<128>  micro_op_resolver;
// static tflite::AllOpsResolver micro_op_resolver;

extern const unsigned char model_data1[];
extern float  test_data[280][3];
extern const float test_Time[280];
extern const float test_v[280];
extern const float test_step[280];

static uint8_t *tflm_area = NULL;               //张量区域

tflm_module_t tflm_soc;


static void AllOpsResolver(tflite::MicroMutableOpResolver<128> *resolver) 
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

/*初�?�化解释�??*/
extern "C"  void tflm_create(tflm_module_t *tflm)
{
    //数据空间判断
    if (tflm_area == NULL || tflm->model_data == NULL) 
    {
        printf("tflm_area don't allocate memory or model_data is null!");
        return;
    }
     //加载模型
    const tflite::Model *tl_model = tflite::GetModel(tflm->model_data);
    if (tl_model->version() != TFLITE_SCHEMA_VERSION) 
    {
        MicroPrintf("Model provided is schema version %d not equal to supported "
    		"version %d.", tl_model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }
    // static tflite::MicroInterpreter interpreter_temp(tl_model, micro_op_resolver, tf_area, TFLM_AREA_SIZE,nullptr,nullptr,true);
    /* 创建解释�?? */
    tflite::MicroInterpreter *interpreter = new tflite::MicroInterpreter(tl_model, micro_op_resolver, tflm_area, TFLM_AREA_SIZE);
    /*分配张量内存*/
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    MicroPrintf("interpreter is regester !");
    if (allocate_status != kTfLiteOk) {
        MicroPrintf("AllocateTensors() failed");
        return;
    }
    // if(interpreter->input_tensor(0)->dims->size != 3)
    // {
    //     MicroPrintf("模型错�??!");
    //     interpreter->~MicroInterpreter();       //释放资源
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
    /*初�?�化张量内存*/
    tflm_area = (uint8_t *)heap_caps_malloc(TFLM_AREA_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    // for(int i = 0;i < 280; i++)
    // {
    //     test_data[i][0] = test_step[i];
    //     test_data[i][1] = test_v[i];
    //     test_data[i][2] = test_Time[i];
    // }

    /*初�?�化模型使用方法*/
    AllOpsResolver(&micro_op_resolver);

    /*初�?�化SOC模型*/
    // tflm_soc.model_data = model_data1;
    // tflm_create(&tflm_soc);
#ifdef USE_SOH_MODEL 
    /*初�?�化SOH模型*/
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
    TfLiteTensor *input  = interpreter->input(0);
    TfLiteTensor *output = interpreter->output(0);
    interpreter->AllocateTensors();

    /*设置输入张量数据*/
    for(int i = 0; i < input_num; i++)
    {
        input->data.f[i] = input_data[i];
    }

    /*进�?�推�??*/
    TfLiteStatus invoke_status = interpreter->Invoke();
    if(invoke_status != kTfLiteOk) 
    {
        MicroPrintf("Invoke failed\n");
        return;
    }

    if(output->type != kTfLiteFloat32)  return;
    int num = output->bytes/4;
    if(num > output_num)    num = output_num;


    /*获取输出张量数据*/
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




