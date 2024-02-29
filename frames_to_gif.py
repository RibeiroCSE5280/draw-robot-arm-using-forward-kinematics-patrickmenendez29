import os
import imageio


def create_gif_from_images(image_folder, output_gif):
    # Get all image files in the folder
    image_files = [os.path.join(image_folder, file) for file in os.listdir(image_folder) if file.endswith('.png') or file.endswith('.jpg')]
    # create a lambda that sorts the files based on a number found in the format frames/frame-<number>.jpg
    image_files.sort(key=lambda x: int(x.split('-')[-1].split('.')[0]))
    
    # Read images into a list
    images = [imageio.imread(file) for file in image_files]

    # Write images to a GIF file
    imageio.mimsave(output_gif, images)

# Usage
create_gif_from_images('frames', 'output.gif')