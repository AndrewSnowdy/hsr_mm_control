# Process Iphone .HEIC to .jpg that can then be imported to roboflow

import os
from PIL import Image
from pillow_heif import register_heif_opener

# Tell Pillow how to handle HEIC
register_heif_opener()

def process_images():
    # Loop through every file in the current folder
    for filename in os.listdir('.'):
        if filename.lower().endswith(".heic"):
            print(f"Fixing {filename}...")
            try:
                img = Image.open(filename)
                
                # 1. Standardize to 640x640 with black padding (Letterbox)
                # This ensures your 'Wave' icon doesn't get squashed
                img.thumbnail((640, 640))
                new_img = Image.new("RGB", (640, 640), (0, 0, 0))
                new_img.paste(img, ((640 - img.size[0]) // 2, (640 - img.size[1]) // 2))
                
                # 2. Save as JPG (This strips the broken metadata)
                clean_name = os.path.splitext(filename)[0] + ".jpg"
                new_img.save(clean_name, "JPEG", quality=95)
                
                # Optional: Remove the original HEIC to save space
                # os.remove(filename) 
                
            except Exception as e:
                print(f"Could not fix {filename}: {e}")

if __name__ == "__main__":
    process_images()
    print("Done! All HEIC files are now 640x640 JPEGs.")