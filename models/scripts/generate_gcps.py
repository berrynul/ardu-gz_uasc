#!/usr/bin/env python3
"""
Generate Gazebo GCP (Ground Control Point) models for the C-UASC localization mission.

Competition spec (Rev 3, 2026-02-23):
  - gcp_large_N  ->  1.2 x 1.2 m  -- four boundary markers that define the search area
  - gcp_small_N  ->  0.6 x 0.6 m  -- numbered identification targets (5-10 placed inside)
  - Pattern      ->  black-and-white 4-triangle X, number centred in red
  - Thickness    ->  0.003 m (3 mm) -- flat but stable in sim

The on-disk model.sdf uses a relative model:// URI so it is not path-dependent.
The C++ spawner (GCPSpawner.cc) builds its own inline SDF with an absolute
file:// URI resolved at runtime via the ament index -- it does NOT read this file.
"""

import os
from PIL import Image, ImageDraw, ImageFont

# Script lives at  <package>/models/scripts/gen_gcps.py
# Models must land in  <package>/models/
# so OUTPUT_DIR is the parent of the scripts/ directory, not a child of it.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR  = os.path.dirname(SCRIPT_DIR)   # one level up: models/
FONT_PATH   = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

GCP_THICKNESS = 0.003   # metres -- 3 mm, flat but not z-fighting thin

# Two size variants matching the competition spec
VARIANTS = {
    "large": 1.2,   # boundary markers
    "small": 0.6,   # numbered identification targets
}

# Texture is always 1024x1024; font size scales with physical tile size so the
# digit occupies the same apparent fraction of the tile at any scale.
TEX_SIZE       = 1024
FONT_SIZE_BASE = int(TEX_SIZE * 0.18)   # for the largest (1.2 m) tile


def make_texture(number: int, side_m: float) -> Image.Image:
    """
    Four-triangle GCP pattern:
      top + bottom wedges -> black
      left + right wedges -> white
      centred digit       -> red

    Font size is scaled proportionally to the physical tile size so the number
    occupies the same fraction of the tile regardless of variant.
    """
    size      = TEX_SIZE
    font_size = int(FONT_SIZE_BASE * (side_m / max(VARIANTS.values())))
    img       = Image.new("RGB", (size, size), "white")
    draw      = ImageDraw.Draw(img)
    cx, cy    = size // 2, size // 2

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

    draw.text((cx, cy), str(number), fill="red", font=font, anchor="mm")
    return img


def make_model(variant: str, number: int) -> None:
    """
    Write one model directory: models/gcp_{variant}_{number}/
      model.config
      model.sdf          <- dimensions baked in, file:// URI for albedo_map
      materials/textures/gcp_{variant}_{number}.png
    """
    side      = VARIANTS[variant]
    name      = f"gcp_{variant}_{number}"
    model_dir = os.path.join(OUTPUT_DIR, name)
    tex_dir   = os.path.join(model_dir, "materials", "textures")
    os.makedirs(tex_dir, exist_ok=True)

    tex_path = os.path.join(tex_dir, f"{name}.png")
    # Relative model:// URI so this file is not broken when the package is
    # installed or the source tree is moved.  The C++ spawner does NOT use
    # this file -- it resolves an absolute path at runtime via ament_index.
    tex_uri  = f"model://{name}/materials/textures/{name}.png"
    half_t   = GCP_THICKNESS / 2.0
    box_str  = f"{side} {side} {GCP_THICKNESS}"

    # -- Texture ---------------------------------------------------------------
    make_texture(number, side).save(tex_path)

    # -- model.config ----------------------------------------------------------
    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    C-UASC localization target -- {variant} GCP #{number}.
    Physical size: {side} m x {side} m x {GCP_THICKNESS} m.
  </description>
</model>
""")

    # -- model.sdf -------------------------------------------------------------
    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="base">
      <!-- z = half thickness so bottom face sits exactly on the ground plane -->
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
            <!-- model:// URI: safe for static loading via GZ_SIM_RESOURCE_PATH.
                 GCPSpawner.cc does NOT read this file; it builds its own
                 inline SDF with a runtime-resolved absolute file:// path. -->
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


# -- Entry point ---------------------------------------------------------------
if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Output -> {OUTPUT_DIR}\n")

    for variant in VARIANTS:
        for n in range(10):
            make_model(variant, n)

    print("\nDone.")
