from abc import ABC, abstractmethod
from typing import TypedDict

import cv2


class BallDetection(TypedDict):
    center: tuple[float, float]
    radius: float


class BallDetector(ABC):
    @abstractmethod
    def detect(self, frame: cv2.typing.MatLike) -> list[BallDetection]: ...
