from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

_cv2_import_error = None
try:
    import cv2  # type: ignore
except Exception as e:  # pragma: no cover - environment-specific
    cv2 = None
    _cv2_import_error = e


@dataclass(frozen=True)
class PadDetection:
    """A single detected colored pad (centroid + size in pixels)."""

    color: str  # 'red' or 'blue'
    centroid_px: Tuple[float, float]  # (u, v)
    area_px: float
    bbox_xywh: Tuple[int, int, int, int]  # (x, y, w, h)
    radius_px: float
    circularity: float
    fill_ratio: float
    confidence: float


class ColorDetector:
    """Pure OpenCV HSV-based detection for red + blue pads."""

    def __init__(
        self,
        thresholds: Dict,
        *,
        morph_kernel_size: int = 5,
        morph_iterations: int = 2,
    ) -> None:
        if cv2 is None:
            raise RuntimeError(
                "OpenCV (cv2) failed to import. "
                "Verify your Python environment has a working OpenCV build "
                "(this environment shows a NumPy/OpenCV binary incompatibility). "
                f"Original error: {_cv2_import_error!r}"
            )

        self.thresholds = thresholds

        red = thresholds.get("red", {})
        blue = thresholds.get("blue", {})
        general = thresholds.get("general", {})

        self.min_contour_area: float = float(general.get("min_contour_area", 500.0))
        self.max_contour_area: float = float(general.get("max_contour_area", 50000.0))
        self.min_circularity: float = float(general.get("min_circularity", 0.5))
        self.min_fill_ratio: float = float(general.get("min_fill_ratio", 0.45))
        self.min_radius_px: float = float(general.get("min_radius_px", 8.0))
        self.pad_real_size_m: float = float(general.get("pad_real_size_m", 1.0))
        self.debug: bool = bool(general.get("debug", False))

        self.red_lower = np.array(red.get("hsv_lower", [0, 100, 100]), dtype=np.uint8)
        self.red_upper = np.array(red.get("hsv_upper", [10, 255, 255]), dtype=np.uint8)
        self.red_lower2 = red.get("hsv_lower2")
        self.red_upper2 = red.get("hsv_upper2")

        self.blue_lower = np.array(blue.get("hsv_lower", [100, 100, 100]), dtype=np.uint8)
        self.blue_upper = np.array(blue.get("hsv_upper", [140, 255, 255]), dtype=np.uint8)

        self.morph_kernel_size = int(general.get("morph_kernel_size", morph_kernel_size))
        self.morph_iterations = int(general.get("morph_iterations", morph_iterations))
        if self.morph_kernel_size >= 3:
            self.kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.morph_kernel_size, self.morph_kernel_size)
            )
        else:
            self.kernel = None

    def _mask_for_red(self, hsv: np.ndarray) -> np.ndarray:
        mask1 = cv2.inRange(hsv, self.red_lower, self.red_upper)
        if self.red_lower2 is None or self.red_upper2 is None:
            return mask1
        lower2 = np.array(self.red_lower2, dtype=np.uint8)
        upper2 = np.array(self.red_upper2, dtype=np.uint8)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        return cv2.bitwise_or(mask1, mask2)

    def _mask_for_blue(self, hsv: np.ndarray) -> np.ndarray:
        return cv2.inRange(hsv, self.blue_lower, self.blue_upper)

    def _postprocess_mask(self, mask: np.ndarray) -> np.ndarray:
        if self.kernel is None:
            return mask

        # Clean small specks and close holes in the pad region.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=self.morph_iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.morph_iterations)
        return mask

    @staticmethod
    def _contours_from_mask(mask: np.ndarray) -> List[np.ndarray]:
        # OpenCV findContours has different return signatures across versions.
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 2:
            contours_list = contours[0]
        else:
            contours_list = contours[1]
        return list(contours_list)

    def _detections_from_mask(self, mask: np.ndarray, *, color_name: str) -> List[PadDetection]:
        detections: List[PadDetection] = []
        for contour in self._contours_from_mask(mask):
            area = float(cv2.contourArea(contour))
            if area < self.min_contour_area or area > self.max_contour_area:
                continue

            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 1e-6:
                continue
            circularity = float((4.0 * np.pi * area) / max(perimeter * perimeter, 1e-6))
            if circularity < self.min_circularity:
                continue

            (circle_cx, circle_cy), radius = cv2.minEnclosingCircle(contour)
            if radius < self.min_radius_px:
                continue

            # Centroid from moments.
            m = cv2.moments(contour)
            if abs(m["m00"]) < 1e-12:
                cx = float(circle_cx)
                cy = float(circle_cy)
            else:
                cx = float(m["m10"] / m["m00"])
                cy = float(m["m01"] / m["m00"])

            x, y, w, h = cv2.boundingRect(contour)
            fill_ratio = float(area / max(float(w * h), 1.0))
            if fill_ratio < self.min_fill_ratio:
                continue

            confidence = float(
                np.clip(
                    0.45 * circularity
                    + 0.25 * fill_ratio
                    + 0.30 * min(area / max(self.min_contour_area * 4.0, 1.0), 1.0),
                    0.0,
                    1.0,
                )
            )
            detections.append(
                PadDetection(
                    color=color_name,
                    centroid_px=(cx, cy),
                    area_px=area,
                    bbox_xywh=(x, y, w, h),
                    radius_px=float(radius),
                    circularity=circularity,
                    fill_ratio=fill_ratio,
                    confidence=confidence,
                )
            )

        # Largest pads first (helps stable tracking if multiple are visible).
        detections.sort(key=lambda d: (d.confidence, d.area_px), reverse=True)
        return detections

    def detect_pads(
        self, bgr_image: np.ndarray, *, return_masks: bool = False
    ) -> Tuple[List[PadDetection], Dict[str, np.ndarray]]:
        """Detect red + blue pads in a BGR image."""
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        mask_red = self._mask_for_red(hsv)
        mask_blue = self._mask_for_blue(hsv)

        mask_red = self._postprocess_mask(mask_red)
        mask_blue = self._postprocess_mask(mask_blue)

        red_dets = self._detections_from_mask(mask_red, color_name="red")
        blue_dets = self._detections_from_mask(mask_blue, color_name="blue")
        detections = red_dets + blue_dets

        debug_masks: Dict[str, np.ndarray] = {}
        if return_masks:
            debug_masks["red"] = mask_red
            debug_masks["blue"] = mask_blue

        return detections, debug_masks
