
# AI開発環境の互換性

このドキュメントは、PyTorch 2.5.1，ensorFlow，MATLABの互換性を保つための説明

## 互換性表

| ソフトウェア       | バージョン       | 対応CUDAバージョン       | 備考                                                                                   |
|------------------|------------------|-------------------------|-----------------------------------------------------------------------------------------|
| **PyTorch**      | 2.5.1            | 11.8, 12.1, 12.4        | PyTorch 2.5.1はCUDA 11.8、12.1、12.4をサポートしています。[PyTorch公式サイト](https://pytorch.org/get-started/locally/) |
| **TensorFlow**   | 2.14.0           | 11.8                    | TensorFlow 2.14.0はCUDA 11.8をサポートしています。                                      |
| **MATLAB**       | R2023b           | 11.8                    | MATLAB R2023bはCUDA 11.8をサポートしています。                                         |

## 推奨構成

すべてのツールでの互換性を確保するため、以下の構成を推奨します：

- **CUDAバージョン:** 11.8
- **cuDNNバージョン:** CUDA 11.8に対応するバージョン（例: cuDNN 8.6）
- **Pythonバージョン:** 3.9、3.10、または3.11

## 注意事項

1. ソフトウェアスタックに基づき、正しいCUDAおよびcuDNNのバージョンをインストールしてください。
2. PyTorchのインストールについては公式サイトを参照してください。
3. MATLAB R2023bでは、CUDA 11.8をサポートしています。GPUを利用するには、Parallel Computing Toolboxが必要です。
4. TensorFlow 2.14.0は、CUDA 11.8およびcuDNN 8.6を必要とします。

## 環境確認方法

### PyTorch
PyTorchでGPUが使用可能か確認する：
```python
import torch
print(torch.cuda.is_available())  # Trueが返されればGPUが使用可能
```
### Tensorflow
TensorFlowでGPUが使用可能か確認する：
```python
import tensorflow as tf
print(tf.config.list_physical_devices('GPU'))  # 利用可能なGPUがリスト表示される
```
### MATLAB
MATLABでGPUが使用可能か確認する：
```matlab
gpuDeviceCount   % GPUの数を表示
gpuDevice(1)     % 最初のGPUの詳細を表示
```
