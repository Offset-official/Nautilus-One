import argparse
import os
from PIL import Image

def convert_to_rgb565(image_path, output_file, width, height, var_name):
    try:
        # Open and validate the image
        try:
            img = Image.open(image_path).convert("RGB")
        except Exception as e:
            print(f"Error opening image: {e}")
            return

        img = img.resize((width, height))
        print(f"Resizing image to {width}x{height}")
        print(f"Resized image size: {img.width}x{img.height}")

        # Convert to RGB565 format
        rgb565_data = []
        for y in range(img.height):
            for x in range(img.width):
                r, g, b = img.getpixel((x, y))
                r5 = (r >> 3) & 0x1F  # 5 bits for red
                g6 = (g >> 2) & 0x3F  # 6 bits for green
                b5 = (b >> 3) & 0x1F  # 5 bits for blue
                rgb565 = (r5 << 11) | (g6 << 5) | b5
                rgb565_data.append(rgb565)
                print(f"Pixel ({x}, {y}): RGB=({r},{g},{b}), RGB565={hex(rgb565)}")

        # Write to output file as binary
        with open(output_file, "wb") as f:
            for pixel in rgb565_data:
                data = pixel.to_bytes(2, byteorder="big")
                f.write(data)
                print(f"Writing pixel data: {data}")

        print(f"Converted image saved to {output_file}")

        # Create a header file with the variable name specified by the user
        header_file = output_file.replace(".rgb565", ".h")
        with open(header_file, "w") as f:
            f.write(f"unsigned char {var_name}[] = {{\n")
            # Convert each pixel to a hexadecimal format and write it as a C array
            for i, pixel in enumerate(rgb565_data):
                f.write(f"0x{pixel:04X}, ")
                if (i + 1) % 8 == 0:  # New line after every 8 pixels for readability
                    f.write("\n")
            f.write("\n};\n")

        print(f"Header file created: {header_file}")

    except Exception as e:
        print(f"Error: {e}")

def main():
    parser = argparse.ArgumentParser(
        description="Convert an image to RGB565 format, resize it, and generate a C header file with a custom variable name."
    )
    parser.add_argument("image_path", help="Path to the input image file")
    parser.add_argument("output_file", help="Path to the output RGB565 binary file")
    parser.add_argument("width", type=int, help="Width of the resized image")
    parser.add_argument("height", type=int, help="Height of the resized image")
    parser.add_argument("var_name", help="Name for the header variable (e.g., 'armed')")

    args = parser.parse_args()

    convert_to_rgb565(args.image_path, args.output_file, args.width, args.height, args.var_name)

if __name__ == "__main__":
    main()
