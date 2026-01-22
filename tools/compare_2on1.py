from __future__ import annotations

import argparse

import cv2
import torch
from t4_devkit import Tier4
from torchmetrics.regression import CosineSimilarity, JensenShannonDivergence

from onnxcomp.onnxcomp.inference import InferenceModel
from onnxcomp.onnxcomp.scorer import Scorer


def inference(onnx_path: str, image_path: str) -> torch.Tensor:
    """Inference ONNX model feature.

    Note:
        Model: Input=(1, 3, H, W) -> Feature=(1, C, H', W')

    Args:
        onnx_path (str): Path to ONNX model.
        image_path (str): Path to image.

    Returns:
        torch.Tensor: Feature probability tensor in the shape of (H', W').
    """
    model = InferenceModel(onnx_path)
    image = cv2.imread(image_path)

    if image is None:
        raise ValueError(f"Failed to read image from {image_path}")

    return model(image)


def compare(onnx1: str, onnx2: str, data_root: str) -> None:
    t4 = Tier4(data_root, verbose=False)

    cossim = CosineSimilarity()
    jsd = JensenShannonDivergence()
    cossim_scorer = Scorer("Cosine Similarity")
    jsd_scorer = Scorer("Jensen-Shannon Divergence")

    for sample_data in t4.sample_data:
        image_path = t4.get_sample_data_path(sample_data.token)

        feat1 = inference(onnx1, image_path)
        feat2 = inference(onnx2, image_path)

        cossim_score = cossim(feat1, feat2).item()
        cossim_scorer.add_score(cossim_score)

        jsd_score = jsd(feat1, feat2).item()
        jsd_scorer.add_score(jsd_score)

        print(f"{cossim_scorer.metric}: {cossim_score:.4f}")
        print(f"{jsd_scorer.metric}: {jsd_score:.4f}")

    cossim_scorer.print_summary()
    jsd_scorer.print_summary()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare 2 ONNX models' features for the same image"
    )
    parser.add_argument("onnx1", type=str, help="Path to ONNX model 1")
    parser.add_argument("onnx2", type=str, help="Path to ONNX model 2")
    parser.add_argument("data_root", type=str, help="Directory path to T4 dataset")
    args = parser.parse_args()

    compare(args.onnx1, args.onnx2, args.data_root)


if __name__ == "__main__":
    main()
