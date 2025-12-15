# MindX SDK
## 初始化
使用前需要初始化
C++
```
//初始化,默认
APP_ERROR ret = MxInit();
//初始化,设置参数
MxBase::AppGlobalCfg globalCfg;
globalCfg.vpcChnNum = 48;//VPC通道,NPU设备内专门处理图像的一个单元
APP_ERROR ret = MxInit(globalCfg); 
..............
//去初始化
ret = MxDeInit();
```
Python
```
from mindx.sdk import base
base.mx_init()
```
## 图像解码与编码
C++
```
//图像处理类
ImageProcessor imageProcessor(deviceId);
//解码后的图像类
Image decodedImage;
//解码
APP_ERROR ret = imageProcessor.Decode(imagePath, decodedImage, ImageFormat::YUV_SP_420);
if (ret != APP_ERR_OK) {
        return 0;
    }

//图像处理操作后图像类
Image resizedImage;
//图像处理操作（缩放）
ret = imageProcessor.Resize(decodedImage, Size(416, 416), resizedImage, Interpolation::HUAWEI_HIGH_ORDER_FILTER);
if (ret != APP_ERR_OK) {
    return 0;
}

//（可选）初始化编码通道JpegEncodeChnConfig jpegEncodeChnConfig;
JpegEncodeChnConfig.maxPicWidth = 4096;
JpegEncodeChnConfig.maxPicHeight = 4096;
imageProcessor.InitJpegEncodeChannel(JpegEncodeChnConfig);

//图像编码
ret = imageProcessor.Encode(resizedImage,"encode.jpg");
if (ret != APP_ERR_OK) {
    return 0;
}
```
Python
```
from mindx.sdk import base  
from mindx.sdk.base import ImageProcessor, Image
# 图像解码
# 初始化ImageProcessor对象
imageProcessor = ImageProcessor(device_id)
# 读取图片路径进行解码，解码格式为nv12（YUV_SP_420）
decoded_image = imageProcessor.decode(image_path, base.nv12)
# 图像处理操作（抠图）  
crop_para = [Rect(300, 100, 550, 350)]  
croped_images = imageProcessor.crop(decoded_image, crop_para)  
image_save_path = "croped_image.jpg"  
# 编码
imageProcessor.encode(croped_images[0], image_save_path) 
```
## 图像处理
C++
抠图
```
Image cropImage;
Rect cropRect {0, 0, 640, 512};
ret = imageProcessor.Crop(decodeImage, cropRect, cropImage);
if (ret != APP_ERR_OK) {
    return 0;
}
```
缩放
```
Image resizedImage;
Size size(416, 416);
ret = imageProcessor.Resize(decodedImage, size, resizedImage, Interpolation::HUAWEI_HIGH_ORDER_FILTER);
if (ret != APP_ERR_OK) {
    return 0;
}
```
补边
```
Image paddingImage;
Dim padDim(0, 0, 240, 240);
Color color(0, 0, 0);
ret = imageProcessor.Padding(decodedImage, padDim, color, BorderType::BORDER_CONSTANT, paddingImage);
if (ret != APP_ERR_OK) {
    return 0;
}
```
抠图缩放(同时执行,提高效率)
```
std::vector<Image> cropResizedImageVec(1);
Rect rect(0, 0, 240, 240);
std::vector<Rect> cropConfigVec = {rect};
Size size(416, 416);

ret = imageProcessor.CropResize(decodedImage, cropConfigVec, size, cropResizedImageVec);
if (ret != APP_ERR_OK) {
    return 0;
}
```
抠图贴图
```
//构造背景Image（也可以选择其他已有数据的Image对象）
Size imageSize(640, 640);
size_t dataSize = 640 * 640 * 3 / 2;
MemoryData imgData(dataSize, MemoryData::MemoryType::MEMORY_DVPP, deviceId);
if (MemoryHelper::Malloc(imgData) != APP_ERR_OK) { 
    return 0;
} 
std::shared_ptr<uint8_t> pastedData((uint8_t*)imgData.ptrData, imgData.free);

/ 抠图贴图操作后图像类
Image pastedImage(pastedData, dataSize, deviceId, imageSize);

//执行抠图贴图
//抠图和贴图尺寸
Rect rectFrom(0, 0, 240, 240);
Rect rectTo(0, 0, 480, 480);
std::pair<Rect, Rect> cropPasteRect = {rectFrom, rectTo};

//抠图贴图操作
ret = imageProcessor.CropAndPaste(resizeImage, cropPasteRect, pastedImage) ;
if (ret != APP_ERR_OK) {
    return 0;
}
```
Python
```
# 图像缩放  
size_para = Size(224, 224)  
resized_image = imageProcessor.resize(decoded_image, size_para, base.huaweiu_high_order_filter)
# 图像补边  
dim_para = Dim(100, 100, 100, 100)  
# 读取将解码后的Image类按Dim进行补边，补边方式为重复最后一个元素  
padded_image = imageProcessor.padding(decoded_image, dim_para, Color(0, 0, 0), base.border_replicate)  
# 图像抠图并缩放  
crop_resize_para = [(Rect(300, 100, 550, 350), Size(100, 100))]  
crop_resize_image = imageProcessor.crop_resize(decoded_image, crop_resize_para) 
# 图像抠图并贴图  
crop_paste_para = (Rect(300, 100, 550, 350), Rect(100, 100, 1500, 1500))  
imageProcessor.crop_paste(decoded_image , crop_paste_para, paste_image)
```
## 视频编码与解码
视频编码需要
定义数据类型
写回调函数
配置编码器
编码
```
// 定义要接收的编码数据
struct FrameImage {
    Image image;                 // Video Image Class
    uint32_t frameId = 0;        // Video Frame Index
    uint32_t channelId = 0;      // Video Channel Index
};

// 定义获取编码数据到自定义结构的回调函数
APP_ERROR CallBackVenc(std::shared_ptr<uint8_t>& outDataPtr, uint32_t& outDataSize, uint32_t& channelId,
                        uint32_t& frameId, void* userData)
{
    Image image(outDataPtr, outDataSize, -1, Size(1920, 1080));
    FrameImage* encodedVec = static_cast<FrameImage*>(userData);
    FrameImage frameImage;
    encodedVec.image = image;
    encodedVec.channelId = channelId;
    encodedVec.frameId = frameId;
    return APP_ERR_OK;
}
// 构造编码配置项
VideoEncodeConfig vEncodeConfig;
VideoEncodeCallBack cPtr2 = CallBackVenc;
vEncodeConfig.callbackFunc = cPtr2;
vEncodeConfig.width = 1920;
vEncodeConfig.height = 1080;
vEncodeConfig.keyFrameInterval = 1;
vEncodeConfig.srcRate = 30;
vEncodeConfig.rcMode = 1;
vEncodeConfig.maxBitRate = 30000;
vEncodeConfig.ipProp = 70;
// 实例化编码类
VideoEncoder videoEncoder(vEncodeConfig, deviceId);
// 执行编码操作
FrameImage encodedFrame;

for (size_t i = 0; i < inputFrameList.size(); i++) {
    ret = videoEncoder.Encode(image, frameId, &encodedFrame);
}
```
解码同理
```
 // 定义要接收的解码数据
    struct FrameImage {
        Image image;                 // Video Image Class
        uint32_t frameId = 0;        // Video Frame Index
        uint32_t channelId = 0;      // Video Channel Index
    };

    //定义获取解码数据到自定义结构的回调函数
    APP_ERROR CallBackVdec(Image& decodedImage, uint32_t channelId, uint32_t frameId, void* userData)
    {
      FrameImage* decodedVec = static_cast<FrameImage*>(userData);
      decodedVec->image = decodedImage;
      decodedVec->channelId = channelId;
      decodedVec->frameId = frameId;
    }
    // 构造解码配置项
    VideoDecodeConfig config;
    VideoDecodeCallBack cPtr = CallBackVdec;
    config.width = 1920;
    config.height = 1080;
    config.callbackFunc = cPtr;
    config.skipInterval = 0;
    config.inputVideoFormat = StreamFormat::H264_MAIN_LEVEL;
    config.outputImageFormat = ImageFormat::YUV_SP_420;
    // 实例化解码类
    VideoDecoder videoDecoder(config, deviceId, channelId);
    // 执行解码操作
    ret = videoDecoder.Decode(dataPtr, dataSize, frameId, &decodedFrame); 
```
Python代码处理逻辑同理,但是简单许多
```
import os
import numpy as np
import time
from mindx.sdk import base
from mindx.sdk.base import Image, ImageProcessor
from mindx.sdk.base import VideoEncoder, VideoEncodeConfig, VencCallBacker 
# 视频编码回调函数  
def venc_callback(pyChar, outDataSize, channelId, frameId):
    with open('video_save_data/output.h264', 'ab') as file:  
        file.write(pyChar)  
 
# 初始化VencCallBacker类并注册回调函数  
vencCallBacker = VencCallBacker()  
vencCallBacker.registerVencCallBack(venc_callback)  
# 初始化VideoEncodeConfig  
venc_conf = VideoEncodeConfig()  
venc_conf.keyFrameInterval = 50  
venc_conf.srcRate = 30  
venc_conf.maxBitRate = 6000  
venc_conf.ipProp = 30  
# 初始化VideoEncoder  
videoEncoder = VideoEncoder(venc_conf, vencCallBacker, device_id)  
# 将编码后数据保存为本地视频，若视频文件已存在则删除  
venc_save_path = os.path.join(save_path, 'output.h264')  
video_encode_exists = os.path.exists(venc_save_path)  
if video_encode_exists:  
     os.remove(venc_save_path)  
# 从decoded_data_list中循环取Image类进行编码  
for i, img in enumerate(decoded_data_list):  
    videoEncoder.encode(img, i) 

import os
import numpy as np
import time
from mindx.sdk import base
from mindx.sdk.base import Image, ImageProcessor
from mindx.sdk.base import VideoDecoder, VideoDecodeConfig, VdecCallBacker
decoded_data_list = []  
# 视频解码回调函数  
def vdec_callback(decodedImage, channelId, frameId):  
    # 解码完成的Image类存入列表中  
    decoded_data_list.append(decodedImage)    
     
# 初始化VdecCallBacker类并注册回调函数  
vdecCallBacker = VdecCallBacker()  
vdecCallBacker.registerVdecCallBack(vdec_callback)  
# 初始化VideoDecodeConfig类并设置参数  
vdecConfig = VideoDecodeConfig()  
vdecConfig.skipInterval = 0  
vdecConfig.inputVideoFormat = base.h264_main_level  
vdecConfig.outputImageFormat = base.nv12  
vdecConfig.width = 1920  
vdecConfig.height = 1080  
# 初始化VideoDecoder  
videoDecoder = VideoDecoder(vdecConfig, vdecCallBacker, device_id, channel_id)  
# 获取需解码视频帧文件名  
srcDataList = ["frame-{}.data".format(i) for i in range(100)]  
# 循环取帧解码  
for i, fileName in enumerate(srcDataList):  
# 读取视频帧数据存入file  
file = np.fromfile(fileName, dtype='uint8')  
# 视频帧数据解码  
videoDecoder.decode(file, i) 
```
## 推理
与pytorch无异,张量,读取模型,推理
```
// 输入二进制数据， 需用户自行准备
std::string filePath = "./test.bin";   
// 读取输入数据到内存                                          
void* dataPtr = ReadTensor(filePath);        
// 输入数据类型，和模型输入数据类型一致                             
auto dataType = MxBase::TensorDType::INT32;     
// 构造输入shape，和模型输入shape一致                 
std::vector<uint32_t> shape = {1, 128};       
// 构造tensor                            
MxBase::Tensor tensor(dataPtr, shape, dataType, 0);   
// 构造模型输入           
std::vector<MxBase::Tensor> inputs{tensor};
// 模型路径，需用户自行指定                           
std::string modelPath = "./test.om";    
// 根据模型路径加载模型                                     
MxBase::Model model(modelPath);           
// 执行模型推理, outputs即为推理结果                                
std::vector<MxBase::Tensor> outputs = model.Infer(inputs);  
```
```
import numpy as np 
from mindx.sdk import base 
from mindx.sdk.base import Tensor, Model
# 模型推理  
# 构造输入Tensor（以二进制输入为例）
# 读取前处理好的numpy array二进制数据   
input_array = np.load("preprocess_array.npy")  
# 构造输入Tensor类并转移至device侧  
input_tensor = Tensor(input_array)  
input_tensor.to_device(device_id)  
# 构造输入Tensor列表  
input_tensors = [input_tensor]  
# 模型路径  
model_path = "resnet50_batchsize_1.om"  
# 初始化Model类  
model = Model(modelPath=model_path, deviceId=device_id)  
# 执行推理  
outputs = model.infer(input_tensors)
```
## 编译
atlas编译前，先设置环境变量
```
source {mxVison安装目录}/mxVision/set_env.sh
```
之后在使用cmake编译