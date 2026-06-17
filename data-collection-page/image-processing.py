from PIL import Image
import numpy as np
import sys
import os

def binarize(input_path: str, output_path: str, threshold: int = 128) -> None:
    img = Image.open(input_path).convert("RGBA")
    data = np.array(img, dtype=np.uint8)

    r, g, b, a = data[..., 0], data[..., 1], data[..., 2], data[..., 3]
    luminance = 0.299 * r + 0.587 * g + 0.114 * b

    # Pixel is "black" when it's opaque AND dark
    is_black = (a > 0) & (luminance < threshold)

    out = np.zeros_like(data)
    out[is_black] = [0, 0, 0, 255]   # black → black, fully opaque
    # everything else stays [0, 0, 0, 0] (transparent)

    Image.fromarray(out, "RGBA").save(output_path)
    print(f"Saved → {output_path}")

    # White-background version: transparent pixels → white
    white_bg = out.copy()
    white_bg[~is_black] = [255, 255, 255, 255]
    base, ext = os.path.splitext(output_path)
    white_path = f"{base}_white{ext}"
    Image.fromarray(white_bg, "RGBA").save(white_path)
    print(f"Saved → {white_path}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python image-processing.py <input.png> [output.png] [threshold=128]")
        sys.exit(1)

    input_path = sys.argv[1]
    base, ext = os.path.splitext(input_path)
    output_path = sys.argv[2] if len(sys.argv) >= 3 else f"{base}_binarized{ext}"
    threshold = int(sys.argv[3]) if len(sys.argv) >= 4 else 200

    binarize(input_path, output_path, threshold)
