import os
import cv2

input_folder = "imgs/"  # Your source folder
output_folder = "converted/"  # Folder for converted PNGs
os.makedirs(output_folder, exist_ok=True)

# File types to convert (non-PNG)
valid_extensions = ['.jpg', '.jpeg', '.bmp', '.tiff', '.webp']

for filename in os.listdir(input_folder):
    name, ext = os.path.splitext(filename)
    ext = ext.lower()

    if ext in valid_extensions:
        input_path = os.path.join(input_folder, filename)
        output_path = os.path.join(output_folder, f"{name}.png")

        img = cv2.imread(input_path)

        if img is not None:
            success = cv2.imwrite(output_path, img)
            if success:
                os.remove(input_path)  # Delete original file
                print(f"Converted and deleted: {filename}")
            else:
                print(f"Failed to save PNG for: {filename}")
        else:
            print(f"Failed to read: {filename}")
