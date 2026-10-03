#include <torch/script.h>
#include <onnxruntime_cxx_api.h>
#include <iostream>
#include <array>
#include <cmath>
int main(int argc, char** argv) try {
 auto t = torch::ones({1, 3});
 std::cout << "Torch sum: " << t.sum().item<float>() << "\n";
 Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
 Ort::SessionOptions options; options.SetIntraOpNumThreads(1);
 if(argc > 1) { Ort::Session session(env, argv[1], options);
 Ort::AllocatorWithDefaultOptions allocator;
 auto input=session.GetInputNameAllocated(0,allocator);
 auto output=session.GetOutputNameAllocated(0,allocator);
 const char* inputs[]={input.get()}; const char* outputs[]={output.get()};
 std::array<float,450> observation{}; std::array<int64_t,2> shape{1,450};
 auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
 auto tensor=Ort::Value::CreateTensor<float>(memory,observation.data(),observation.size(),shape.data(),shape.size());
 auto result=session.Run(Ort::RunOptions{nullptr},inputs,&tensor,1,outputs,1);
 if(result[0].GetTensorTypeAndShapeInfo().GetElementCount()!=12) return 2;
 for(int i=0;i<12;i++) if(!std::isfinite(result[0].GetTensorData<float>()[i])) return 3;
 std::cout << "ONNX inference: 450 observations -> 12 finite actions\n"; }
 if(argc > 2) { auto policy=torch::jit::load(argv[2]); std::cout << "TorchScript loaded\n"; }
} catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
