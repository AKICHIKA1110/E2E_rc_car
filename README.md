# AI Development Environment Compatibility

This document provides a compatibility table for PyTorch 2.5.1, TensorFlow 2.14.0, and MATLAB R2023b, along with their corresponding CUDA and cuDNN versions.

## Compatibility Table

| Software         | Version          | Supported CUDA Versions | Notes                                                                                   |
|------------------|------------------|-------------------------|-----------------------------------------------------------------------------------------|
| **PyTorch**      | 2.5.1            | 11.8, 12.1, 12.4        | PyTorch 2.5.1 supports CUDA 11.8, 12.1, and 12.4. [PyTorch Website](https://pytorch.org/get-started/locally/) |
| **TensorFlow**   | 2.14.0           | 11.8                    | TensorFlow 2.14.0 supports CUDA 11.8.                                                   |
| **MATLAB**       | R2023b           | 11.8                    | MATLAB R2023b supports CUDA 11.8.                                                      |

## Recommended Configuration

To ensure compatibility across all tools, the following setup is recommended:

- **CUDA Version:** 11.8
- **cuDNN Version:** Compatible with CUDA 11.8 (e.g., cuDNN 8.6)
- **Python Version:** 3.9, 3.10, or 3.11

## Notes

1. Ensure the correct version of CUDA and cuDNN is installed based on your software stack.
2. For PyTorch installation.
3. MATLAB R2023b supports CUDA 11.8; ensure you have Parallel Computing Toolbox installed for GPU support.
4. TensorFlow 2.14.0 requires CUDA 11.8 and cuDNN 8.6 for GPU acceleration.

## Setup Verification

### PyTorch
To verify GPU support in PyTorch:
```python
import torch
print(torch.cuda.is_available())  # Should return True
```
### Tensorflow
To verify GPU support in Tensorflow:
```python
import tensorflow as tf
print(tf.config.list_physical_devices('GPU'))  # Should list available GPUs
```
