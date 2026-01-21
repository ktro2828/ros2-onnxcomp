from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Scorer:
    """Scorer class for aggregating scores.

    Attributes:
        metric (str): The metric name.
        scores (list[float]): The list of scores.
    """

    metric: str
    scores: list[float] = field(init=False, default_factory=list)

    def add_score(self, score: float) -> None:
        self.scores.append(score)

    def mean(self) -> float:
        return np.mean(self.scores).item()

    def std(self) -> float:
        return np.std(self.scores).item()

    def median(self) -> float:
        return np.median(self.scores).item()

    def max(self) -> float:
        return np.max(self.scores).item()

    def min(self) -> float:
        return np.min(self.scores).item()

    def percentile(self, p: float) -> float:
        return np.percentile(self.scores, p).item()

    def print_summary(self) -> None:
        print(f"=== Summary: {self.metric} ===")
        print(f"- Mean: {self.mean():.4f}")
        print(f"- Std Dev: {self.std():.4f}")
        print(f"- Median: {self.median():.4f}")
        print(f"- Min: {self.min():.4f}")
        print(f"- Max: {self.max():.4f}")
        print(f"- 99%ile: {self.percentile(99):.4f}")
