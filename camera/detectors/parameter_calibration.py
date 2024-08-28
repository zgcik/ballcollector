import cv2
import numpy as np


def set_h(x, self):
    self.h = x


def set_s(x, self):
    self.s = x


def set_v(x, self):
    self.v = x


def set_value(x, self):
    self.value = x


class HsvCalibrator:
    h: int
    s: int
    v: int
    name: str

    def __init__(
        self, name: str, show: bool, initial_values: tuple[int, int, int], prefix=""
    ) -> None:
        self.name = name
        self.h = initial_values[0]
        self.s = initial_values[1]
        self.v = initial_values[2]
        if show:
            cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
            cv2.createTrackbar(
                f"{prefix} H", name, self.h, 179, lambda x: set_h(x, self)
            )
            cv2.createTrackbar(
                f"{prefix} S", name, self.s, 255, lambda x: set_s(x, self)
            )
            cv2.createTrackbar(
                f"{prefix} V", name, self.v, 255, lambda x: set_v(x, self)
            )

    def value(self):
        return np.array([self.h, self.s, self.v])


class HsvRangeCalibrator:
    lower: HsvCalibrator
    upper: HsvCalibrator

    def __init__(
        self,
        name: str,
        show: bool,
        initial_lower: tuple[int, int, int],
        initial_upper: tuple[int, int, int],
    ) -> None:
        self.lower = HsvCalibrator(name, show, initial_lower, "lower")
        self.upper = HsvCalibrator(name, show, initial_upper, "upper")


class ParameterCalibrator:
    value: int

    def __init__(self, name, show, initial_value: int, prefix="", max_val=255) -> None:
        self.value = initial_value
        if show:
            cv2.namedWindow(name)
            cv2.createTrackbar(
                f"{prefix} val", name, self.value, max_val, lambda x: set_value(x, self)
            )
