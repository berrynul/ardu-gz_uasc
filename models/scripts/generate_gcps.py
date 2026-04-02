#!/usr/bin/env python3
"""
Generate Gazebo GCP (Ground Control Point) models for the C-UASC localization mission.

Competition spec (Rev 3, 2026-02-23):
  - gcp_large     ->  1.2 x 1.2 m  -- boundary markers (no number)
  - gcp_small_N   ->  0.6 x 0.6 m  -- numbered identification targets (5-10 placed inside)
  - Pattern       ->  black-and-white 4-triangle X
  - Small number  ->  black, underlined, centred in the right triangle, upright (not rotated)
  - Thickness     ->  0.003 m (3 mm) -- flat but stable in sim

The on-disk model.sdf uses a relative model:// URI so it is not path-dependent.
The C++ spawner (GCPSpawner.cc) builds its own inline SDF with an absolute
file:// URI resolved at runtime via GZ_SIM_RESOURCE_PATH.
"""

import os
from PIL import Image, ImageDraw, ImageFont

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR  = os.path.dirname(SCRIPT_DIR)   # one level up: models/
FONT_PATH   = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

GCP_THICKNESS = 0.003   # metres

LARGE_SIDE = 1.2
SMALL_SIDE = 0.6

TEX_SIZE       = 1024
FONT_SIZE_BASE = int(TEX_SIZE * 0.18)


def make_large_texture() -> Image.Image:
    """
    Four-triangle GCP pattern with no number:
      top + bottom wedges -> black
      left + right wedges -> white
    """
    size = TEX_SIZE
    img  = Image.new("RGB", (size, size), "white")
    draw = ImageDraw.Draw(img)
    cx, cy = size // 2, size // 2

    draw.polygon([(0, 0),    (size, 0),    (cx, cy)], fill="black")   # top
    draw.polygon([(0, size), (size, size), (cx, cy)], fill="black")   # bottom
    draw.polygon([(0, 0),    (0, size),    (cx, cy)], fill="white")   # left
    draw.polygon([(size, 0), (size, size), (cx, cy)], fill="white")   # right

    draw.rectangle([0, 0, size - 1, size - 1],
                   outline="black", width=max(2, size // 128))
    return img


def make_small_texture(number: int) -> Image.Image:
    """
    Four-triangle GCP pattern with a numbered label:
      top + bottom wedges -> black
      left + right wedges -> white
      number              -> black, underlined, upright, centred in the right triangle,
                             twice the base font size
    """
    size = TEX_SIZE
    # Font is 2x the base size, scaled for the small variant
    font_size = int(FONT_SIZE_BASE * 2 * (SMALL_SIDE / LARGE_SIDE))
    img  = Image.new("RGB", (size, size), "white")
    draw = ImageDraw.Draw(img)
    cx, cy = size // 2, size // 2

    draw.polygon([(0, 0),    (size, 0),    (cx, cy)], fill="black")   # top
    draw.polygon([(0, size), (size, size), (cx, cy)], fill="black")   # bottom
    draw.polygon([(0, 0),    (0, size),    (cx, cy)], fill="white")   # left
    draw.polygon([(size, 0), (size, size), (cx, cy)], fill="white")   # right

    draw.rectangle([0, 0, size - 1, size - 1],
                   outline="black", width=max(2, size // 128))

    try:
        font = ImageFont.truetype(FONT_PATH, size=font_size)
    except IOError:
        font = ImageFont.load_default()

    # Centre of the right triangle: centroid of (cx,cy), (size,0), (size,size)
    rx = int((cx + size + size) / 3)
    ry = cy  # vertically centred

    # Draw the number (upright, black)
    text = str(number)
    draw.text((rx, ry), text, fill="black", font=font, anchor="mm")

    # Draw underline beneath the number
    bbox = font.getbbox(text, anchor="mm")
    # bbox is (left, top, right, bottom) relative to anchor point
    underline_y = ry + bbox[3] + max(2, font_size // 15)
    half_w = (bbox[2] - bbox[0]) / 2
    line_thickness = max(3, font_size // 10)
    draw.line(
        [(rx - half_w, underline_y), (rx + half_w, underline_y)],
        fill="black", width=line_thickness
    )

    return img


def write_model(name: str, side: float, tex_img: Image.Image) -> None:
    """
    Write one model directory: models/{name}/
      model.config
      model.sdf
      materials/textures/{name}.png
    """
    model_dir = os.path.join(OUTPUT_DIR, name)
    tex_dir   = os.path.join(model_dir, "materials", "textures")
    os.makedirs(tex_dir, exist_ok=True)

    tex_path = os.path.join(tex_dir, f"{name}.png")
    tex_uri  = f"model://{name}/materials/textures/{name}.png"
    half_t   = GCP_THICKNESS / 2.0
    box_str  = f"{side} {side} {GCP_THICKNESS}"

    tex_img.save(tex_path)

    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    C-UASC localization target -- {name}.
    Physical size: {side} m x {side} m x {GCP_THICKNESS} m.
  </description>
</model>
""")

    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="base">
      <pose>0 0 {half_t} 0 0 0</pose>

      <collision name="collision">
        <geometry><box><size>{box_str}</size></box></geometry>
      </collision>

      <visual name="visual">
        <geometry><box><size>{box_str}</size></box></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr><metal>
            <albedo_map>{tex_uri}</albedo_map>
            <metalness>0.0</metalness>
            <roughness>1.0</roughness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""")

    print(f"  {name}/  {side} m x {side} m x {GCP_THICKNESS} m")


if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Output -> {OUTPUT_DIR}\n")

    # Single large GCP model (no number)
    write_model("gcp_large", LARGE_SIDE, make_large_texture())

    # 10 numbered small GCP models
    for n in range(10):
        write_model(f"gcp_small_{n}", SMALL_SIDE, make_small_texture(n))

    print("\nDone.")
