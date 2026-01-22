"""Exports a YOLOv5 *.pt model to ONNX and TorchScript formats

Usage:
    $ export PYTHONPATH="$PWD" && python models/export.py --weights ./weights/yolov5s.pt --img 640 --batch 1
"""

import argparse
import os
import sys
import logging  # 导入 logging 模块

# 使用绝对路径import，对应教程的 export PYTHONPATH="$PWD"
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from models.common import *
from utils import google_utils

# 设置 LOGGER
logging.basicConfig(level=logging.INFO)  # 设置日志级别
LOGGER = logging.getLogger(__name__)  # 创建 LOGGER

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='./yolov5s.pt', help='weights path')
    parser.add_argument('--img-size', nargs='+', type=int, default=[640, 640], help='image size')
    parser.add_argument('--batch-size', type=int, default=1, help='batch size')
    opt = parser.parse_args()
    opt.img_size *= 2 if len(opt.img_size) == 1 else 1  # expand
    LOGGER.info(opt)  # 使用 LOGGER 记录信息

    # Input
    img = torch.zeros((opt.batch_size, 3, *opt.img_size))  # image size(1,3,320,192) iDetection

    # Load PyTorch model
    google_utils.attempt_download(opt.weights)
    model = torch.load(
        opt.weights,
        map_location=torch.device('cpu'),
        weights_only=False,  # 👈 显式关掉“只加载权重”的安全模式
    )['model'].float()

    model.eval()
    model.model[-1].export = True  # set Detect() layer export=True
    y = model(img)  # dry run

    # TorchScript export
    try:
        LOGGER.info('\nStarting TorchScript export with torch %s...' % torch.__version__)
        f = opt.weights.replace('.pt', '.torchscript.pt')  # filename
        ts = torch.jit.trace(model, img)
        ts.save(f)
        LOGGER.info('TorchScript export success, saved as %s' % f)
    except Exception as e:
        LOGGER.error('TorchScript export failure: %s' % e)

    # ONNX export
    try:
        import onnx

        LOGGER.info('\nStarting ONNX export with onnx %s...' % onnx.__version__)
        f = opt.weights.replace('.pt', '.onnx')  # filename
        model.fuse()  # only for ONNX
        torch.onnx.export(model, img, f, verbose=False, opset_version=12, input_names=['images'],
                          output_names=['classes', 'boxes'] if y is None else ['output'])

        # Checks
        onnx_model = onnx.load(f)  # load onnx model
        onnx.checker.check_model(onnx_model)  # check onnx model
        
        # 保存为单一文件（将外部数据合并到模型中）
        onnx.save_model(onnx_model, f, save_as_external_data=False)
        LOGGER.info('ONNX export success, saved as %s' % f)
    except Exception as e:
        LOGGER.error('ONNX export failure: %s' % e)

    # CoreML export
    try:
        import coremltools as ct

        LOGGER.info('\nStarting CoreML export with coremltools %s...' % ct.__version__)
        # convert model from torchscript and apply pixel scaling as per detect.py
        model = ct.convert(ts, inputs=[ct.ImageType(name='images', shape=img.shape, scale=1 / 255.0, bias=[0, 0, 0])])
        f = opt.weights.replace('.pt', '.mlmodel')  # filename
        model.save(f)
        LOGGER.info('CoreML export success, saved as %s' % f)
    except Exception as e:
        LOGGER.error('CoreML export failure: %s' % e)

    # Finish
    LOGGER.info('\nExport complete. Visualize with https://github.com/lutzroeder/netron.')