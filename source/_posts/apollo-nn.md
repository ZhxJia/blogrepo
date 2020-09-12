---

title: Apollo中的神经网络框架
categories:
- autonomous
- apollo
tags:
- apollo
mathjax: true
---

### 模型类型

model_type

`modules/perception/inference/inference_factory.h`

```c++
Inference *CreateInferenceByName(const std::string &name,
                                 const std::string &proto_file,
                                 const std::string &weight_file,
                                 const std::vector<std::string> &outputs,
                                 const std::vector<std::string> &inputs,
                                 const std::string &model_root) {
  if (name == "CaffeNet") {
    return new CaffeNet(proto_file, weight_file, outputs, inputs);
  } else if (name == "RTNet") {
    return new RTNet(proto_file, weight_file, outputs, inputs);
  } else if (name == "RTNetInt8") {
    return new RTNet(proto_file, weight_file, outputs, inputs, model_root);
  } else if (name == "PaddleNet") {
    return new PaddleNet(proto_file, weight_file, outputs, inputs);
  }
  return nullptr;
}
```

RTNet 表示使用TensorRT加速