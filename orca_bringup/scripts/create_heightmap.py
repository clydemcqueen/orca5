#!/usr/bin/env python3

# Usage:
# ./orca_bringup/scripts/create_heightmap.py
# mv heightmap.png orca_bringup/models/seafloor

import numpy as np
from PIL import Image

# Must be a power of 2 for Ogre
WIDTH = 128
HEIGHT = 128
FILENAME = 'heightmap.png'

# List of feature sizes. Larger numbers create larger features (big hills),
# smaller numbers create smaller details.
OCTAVES = [64, 32, 16, 8]

# How much each successive octave contributes. 0.5 means each octave
# is half as strong as the one before it.
PERSISTENCE = 0.5


def generate_noise_layer(width, height, feature_size):
    """
    Generate a single layer of smooth noise.

    1. Create a small grid of random values.
    2. Scale this small grid up to the full image size using smooth interpolation.
    """
    # Calculate the size of the small grid
    # Ensure at least 2x2 grid for interpolation to work well
    low_res_w = max(2, width // feature_size)
    low_res_h = max(2, height // feature_size)

    # Generate random noise (values between 0.0 and 1.0)
    noise = np.random.rand(low_res_h, low_res_w).astype(np.float32)

    # Convert the numpy array to a PIL Image in 32-bit float mode
    img_low_res = Image.fromarray(noise, mode='F')

    # Resize the small image to the target size using bicubic interpolation
    # This creates the smooth, rolling effect.
    img_high_res = img_low_res.resize((width, height), Image.Resampling.BICUBIC)

    # Convert the resized image back to a numpy array
    return np.array(img_high_res)


def generate_fractal_noise(width, height, octaves, persistence):
    """
    Generate fractal noise by combining multiple noise layers (octaves).
    """
    final_heightmap = np.zeros((height, width), dtype=np.float32)
    amplitude = 1.0
    total_amplitude = 0.0

    for i, size in enumerate(octaves):
        print(f"  - Layer {i + 1}/{len(octaves)} (feature size: {size}x{size})")
        # Add a noise layer, scaled by the current amplitude
        final_heightmap += generate_noise_layer(width, height, size) * amplitude

        # Keep track of the total amplitude for normalization
        total_amplitude += amplitude

        # Reduce amplitude for the next, more detailed layer
        amplitude *= persistence

    # Normalize the heightmap to be in the range [0.0, 1.0]
    # This prevents bright/dark spots if amplitudes add up to > 1.0
    if total_amplitude > 0:
        normalized_map = final_heightmap / total_amplitude
    else:
        normalized_map = final_heightmap

    # Clamp values just in case interpolation produced something slightly
    # out of bounds (though it shouldn't with bicubic on 0-1 range)
    normalized_map = np.clip(normalized_map, 0.0, 1.0)

    return normalized_map


def main():
    print(f"Create {WIDTH}x{HEIGHT} heightmap...")

    # Generate the fractal noise data (float values 0.0 - 1.0)
    heightmap_data = generate_fractal_noise(WIDTH, HEIGHT, OCTAVES, PERSISTENCE)

    # Scale data to 8-bit integer range (0 - 255)
    img_data = (heightmap_data * 255).astype(np.uint8)

    # Create a grayscale ('L') image from the data
    img = Image.fromarray(img_data, mode='L')

    print(f"\nSave to '{FILENAME}'")
    
    img.save(FILENAME)


if __name__ == "__main__":
    # Ensure you have the required libraries:
    # pip install numpy pillow
    main()
