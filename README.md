## Raspberry Pi に Ubuntu 22.04 を使用して ROS 2 Humble Hawksbill をインストール

このガイドでは、Raspberry Pi 上で Ubuntu 22.04 を使用し、ROS 2 Humble Hawksbill をインストールする手順を説明します。

---

## 1. システムの準備

### システムのアップデートと必要なツールのインストール
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl wget gnupg lsb-release build-essential
```

### curl と wget のインストールについて
`curl` と `wget` は、ウェブからファイルを取得するためのコマンドラインツールです。これらは ROS 2 のキーやその他のリソースを取得する際に必要となります。上記のコマンドで同時にインストールされます。

### ROS 2 のキーを追加
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### ROS 2 リポジトリの設定
```bash
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

## 2. ROS 2 のインストール

### パッケージリストの更新
```bash
sudo apt update
```

### ROS 2 Desktop（フルパッケージ）のインストール
```bash
sudo apt install -y ros-humble-desktop
```

> **注意:** デスクトップ環境を必要としない場合は、以下のコマンドで基本パッケージをインストールできます：
> ```bash
> sudo apt install ros-humble-ros-base
> ```

---

## 3. 環境設定

### ROS 2 のセットアップを `bashrc` に追加
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. colcon のインストール（任意）

`colcon` は ROS 2 のワークスペースをビルドするために必要です。

### colcon のインストール
```bash
sudo apt install -y python3-colcon-common-extensions
```
---

## 5. Visual Studio Code (ARM版) のインストール

Raspberry Pi 用の Visual Studio Code をインストールするには、以下の手順を実行します。

### ARM 用の .deb パッケージをダウンロード
Visual Studio Code (ARM版) の .deb パッケージを [公式サイト](https://code.visualstudio.com/) から取得してください。

### インストール
ダウンロードしたファイルが例えば `~/Downloads/code-arm64.deb` にある場合、以下のコマンドを実行します：

```bash
sudo dpkg -i ~/Downloads/code-arm64.deb
sudo apt --fix-broken install
```

これで Visual Studio Code のインストールが完了します。

---

これで ROS 2 の環境設定が完了しました。問題が発生した場合は、公式の [ROS 2 ドキュメント](https://docs.ros.org/en/humble/index.html) を参照してください。

# AI開発環境の互換性

このドキュメントは、PyTorch 2.5.1，tensorFlow，MATLABの互換性を保つための説明

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
