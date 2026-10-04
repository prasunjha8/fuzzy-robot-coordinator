#!/usr/bin/env python3
"""Render an animated MuJoCo warehouse demo for the Markdown documentation."""

import argparse
import math
import sys
from pathlib import Path

import mujoco
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mujoco_sim import (
    SIMULATION_DT,
    build_world_xml,
    set_robot_controls,
    sync_state_from_physics,
    sync_task_markers,
)
from simulator import MAX_TASKS, SimulationState
ASSET_DIR = ROOT / "docs" / "assets"


def load_font(size):
    font_paths = [
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for font_path in font_paths:
        try:
            return ImageFont.truetype(font_path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def draw_overlay(frame, state, frame_index, frame_count):
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image, "RGBA")
    title_font = load_font(14)
    body_font = load_font(12)
    left = image.width - 278
    top = image.height - 116
    draw.rounded_rectangle((left, top, image.width - 12, image.height - 12), radius=12, fill=(5, 12, 22, 205))
    draw.text((left + 12, top + 8), "FUZZY WAREHOUSE / MuJoCo", font=title_font, fill=(235, 244, 255, 255))
    draw.text((left + 12, top + 31), "Cyan dock · gold open · green done", font=body_font, fill=(190, 211, 231, 255))
    draw.text((left + 12, top + 49), f"Physics sample {frame_index + 1}/{frame_count}", font=body_font, fill=(190, 211, 231, 255))
    for offset, (name, robot) in enumerate(state.robots.items()):
        task_text = f"charging {robot['battery']:.0f}%" if robot["charging"] else (robot["task"] or robot["status"])
        draw.text((left + 12, top + 68 + offset * 14), f"{name}: {task_text}", font=body_font, fill=(230, 238, 248, 255))
    return image


def render_demo(frame_count=144, steps_per_frame=70, width=720, height=450):
    if frame_count < 2:
        raise ValueError("frame_count must be at least two")
    if steps_per_frame < 1:
        raise ValueError("steps_per_frame must be positive")

    state = SimulationState()
    model = mujoco.MjModel.from_xml_string(build_world_xml(state))
    data = mujoco.MjData(model)
    sync_task_markers(model, state)
    mujoco.mj_forward(model, data)

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.lookat[:] = (5.0, 5.0, 0.0)
    camera.distance = 14.0
    camera.azimuth = 90
    camera.elevation = -87

    previous_positions = {
        name: (robot["x"], robot["y"]) for name, robot in state.robots.items()
    }
    initial_positions = dict(previous_positions)
    frames = []
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        for frame_index in range(frame_count):
            for _ in range(steps_per_frame):
                state.tick(dt=SIMULATION_DT, move_robots=False)
                sync_task_markers(model, state)
                mujoco.mj_forward(model, data)
                set_robot_controls(model, data, state)
                mujoco.mj_step(model, data)
                sync_state_from_physics(model, data, state, previous_positions)

            renderer.update_scene(data, camera=camera)
            frame = renderer.render()
            frames.append(draw_overlay(frame, state, frame_index, frame_count))
    finally:
        renderer.close()

    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    gif_path = ASSET_DIR / "mujoco-warehouse-demo.gif"
    screenshot_path = ASSET_DIR / "mujoco-warehouse-demo.png"
    paletted_frames = [
        frame.convert("P", palette=Image.Palette.ADAPTIVE, colors=128)
        for frame in frames
    ]
    paletted_frames[0].save(
        gif_path,
        save_all=True,
        append_images=paletted_frames[1:],
        duration=110,
        loop=0,
        optimize=True,
        disposal=2,
    )
    frames[min(24, len(frames) - 1)].save(screenshot_path, optimize=True)
    moved = any(
        math.hypot(
            state.robots[name]["x"] - initial_positions[name][0],
            state.robots[name]["y"] - initial_positions[name][1],
        )
        > 0.05
        for name in state.robots
    )
    if not moved:
        raise RuntimeError("Generated demonstration has no robot motion.")
    return gif_path, screenshot_path, state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=144)
    parser.add_argument("--steps-per-frame", type=int, default=70)
    parser.add_argument("--width", type=int, default=720)
    parser.add_argument("--height", type=int, default=450)
    args = parser.parse_args()
    if len(SimulationState().tasks) > MAX_TASKS:
        raise RuntimeError("Seed tasks exceed the MuJoCo task-marker capacity.")

    gif_path, screenshot_path, state = render_demo(
        args.frames, args.steps_per_frame, args.width, args.height
    )
    print(f"Generated GIF: {gif_path.relative_to(ROOT)}")
    print(f"Generated still: {screenshot_path.relative_to(ROOT)}")
    print(f"Completed jobs: {len(state.completed)}")


if __name__ == "__main__":
    main()
