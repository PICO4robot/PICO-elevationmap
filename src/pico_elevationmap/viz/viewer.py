"""Single-window OpenCV viewer combining 2D layers and the 3D voxel panel."""

from __future__ import annotations

import math

import numpy as np

from pico_elevationmap.viz.render import render_layer, render_legend
from pico_elevationmap.viz.voxel import render_voxels

__all__ = ["SingleWindowViewer"]


class SingleWindowViewer:
    """合并 2D 高程/可通行性/色标与 3D 体素视图，只创建一个 HighGUI 窗口。"""

    def __init__(
        self,
        *,
        show_2d: bool = True,
        show_3d: bool = True,
        z_min: float = -0.1,
        z_max: float = 1.0,
        bands: int = 12,
        voxel_size: float = 0.05,
        voxel_alpha: float = 0.55,
        title: str = "PICO elevation map",
        width: int = 1280,
        height: int = 900,
    ) -> None:
        if not (show_2d or show_3d):
            raise ValueError("at least one of show_2d/show_3d must be True")
        import cv2

        self._cv2 = cv2
        self.show_2d = show_2d
        self.show_3d = show_3d
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.bands = int(bands)
        self.voxel_size = float(voxel_size)
        self.voxel_alpha = float(voxel_alpha)
        self.title = title
        self._state = {
            "yaw": math.radians(-10.0),
            "pitch": math.radians(38.0),
            "distance": 6.0,
            "drag": None,
        }
        self.last_voxel_panel: np.ndarray | None = None
        self.last_canvas: np.ndarray | None = None
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(title, width, height)
        cv2.setMouseCallback(title, self._on_mouse)

    def _on_mouse(self, event: int, x: int, y: int, flags: int, _param: object) -> None:
        cv2 = self._cv2
        if not self.show_3d:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self._state["drag"] = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self._state["drag"] is not None:
            if not (flags & cv2.EVENT_FLAG_LBUTTON):
                self._state["drag"] = None
                return
            last_x, last_y = self._state["drag"]
            self._state["yaw"] += (x - last_x) * 0.01
            self._state["pitch"] = float(np.clip(self._state["pitch"] + (last_y - y) * 0.01, 0.05, 1.4))
            self._state["drag"] = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self._state["drag"] = None
        elif event == cv2.EVENT_MOUSEWHEEL:
            step = -0.5 if flags > 0 else 0.5
            self._state["distance"] = float(np.clip(self._state["distance"] + step, 1.0, 20.0))

    def compose(
        self,
        *,
        elevation: np.ndarray,
        traversability: np.ndarray | None,
        voxel_centers: np.ndarray,
        voxel_probabilities: np.ndarray | None = None,
        target: np.ndarray,
        fps: float,
        coverage: float,
        resolution: float,
    ) -> np.ndarray:
        cv2 = self._cv2
        top: np.ndarray | None = None
        bottom: np.ndarray | None = None

        if self.show_2d:
            elev_img = render_layer(
                elevation,
                colormap="turbo",
                z_min=self.z_min,
                z_max=self.z_max,
                bands=self.bands,
                resolution=resolution,
            )
            panels = [elev_img]
            if traversability is not None:
                panels.append(
                    render_layer(
                        traversability,
                        colormap="viridis",
                        z_min=0.0,
                        z_max=1.0,
                        bands=2,
                        shade=False,
                    )
                )
            panels.append(
                render_legend(
                    self.z_min,
                    self.z_max,
                    height=elev_img.shape[0],
                    bands=self.bands,
                )
            )
            top = np.hstack(panels)

        if self.show_3d:
            voxel_img = render_voxels(
                voxel_centers,
                probabilities=voxel_probabilities,
                z_min=self.z_min,
                z_max=self.z_max,
                target=target,
                yaw=self._state["yaw"],
                pitch=self._state["pitch"],
                distance=self._state["distance"],
                voxel_size=self.voxel_size,
                bands=self.bands,
                alpha=self.voxel_alpha,
            )
            legend = render_legend(
                self.z_min,
                self.z_max,
                height=voxel_img.shape[0],
                bands=self.bands,
            )
            bottom = np.hstack([voxel_img, legend])
            self.last_voxel_panel = bottom

        if top is not None and bottom is not None:
            new_w = bottom.shape[1]
            new_h = max(1, int(round(top.shape[0] * new_w / top.shape[1])))
            top = cv2.resize(top, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
            canvas = np.vstack([top, bottom])
        elif top is not None:
            canvas = top
        else:
            assert bottom is not None
            canvas = bottom

        if self.show_2d:
            cv2.putText(
                canvas,
                f"elevation + traversability | {fps:5.1f} fps | coverage {coverage:5.1%}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        self.last_canvas = canvas
        return canvas

    def show(self, canvas: np.ndarray) -> None:
        self._cv2.imshow(self.title, canvas)

    def wait_key(self) -> bool:
        """Return True when the user asks to quit (q/Esc)."""
        cv2 = self._cv2
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            return True
        if self.show_3d:
            if key == ord("a"):
                self._state["yaw"] -= 0.1
            elif key == ord("d"):
                self._state["yaw"] += 0.1
            elif key == ord("w"):
                self._state["pitch"] = min(self._state["pitch"] + 0.08, 1.4)
            elif key == ord("s"):
                self._state["pitch"] = max(self._state["pitch"] - 0.08, 0.05)
            elif key in (ord("+"), ord("=")):
                self._state["distance"] = max(1.0, self._state["distance"] - 0.5)
            elif key in (ord("-"), ord("_")):
                self._state["distance"] = min(20.0, self._state["distance"] + 0.5)
        return False

    def save_voxel(self, path: str) -> bool:
        if self.last_voxel_panel is None:
            return False
        self._cv2.imwrite(path, self.last_voxel_panel)
        return True

    def save_canvas(self, path: str) -> bool:
        if self.last_canvas is None:
            return False
        self._cv2.imwrite(path, self.last_canvas)
        return True

    def close(self) -> None:
        self._cv2.destroyAllWindows()
