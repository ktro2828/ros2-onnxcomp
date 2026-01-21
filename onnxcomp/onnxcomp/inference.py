from __future__ import annotations

import cv2
import numpy as np
import onnxruntime as ort
import torch
import torch.nn.functional as F


class InferenceModel:
    def __init__(self, onnx_path: str) -> None:
        self._session = ort.InferenceSession(
            onnx_path, providers=["CPUExecutionProvider", "CUDAExecutionProvider"]
        )

        in_metadata = self._session.get_inputs()[0]
        self._in_shape = in_metadata.shape
        self._in_name = in_metadata.name
        self._out_names = [o.name for o in self._session.get_outputs()]

    def __call__(self, image: np.ndarray) -> torch.Tensor:
        """Inference ONNX model feature.

        Note:
            Model: Input=(1, 3, H, W) -> Feature=(1, C, H', W')

        Args:
            onnx_path (str): Path to ONNX model.
            image_path (str): Path to image.

        Returns:
            torch.Tensor: Feature probability tensor in the shape of (H', W').
        """
        # preprocess image
        image = cv2.resize(image, (self._in_shape[3], self._in_shape[2]))
        image = image.astype(np.float32) / 255.0
        image = np.expand_dims(image, axis=0)
        image = np.transpose(image, (0, 3, 1, 2))

        # inference
        *_, feature = self._session.run(self._out_names, {self._in_name: image})
        feature = np.asarray(feature)
        feature = torch.from_numpy(feature).unsqueeze(0)
        p = F.softmax(feature, dim=1)
        return p
