import argparse
import json
import os
import sys
from io import BytesIO
import matplotlib.pyplot as plt
import numpy as np
import yaml
import yourdfpy
from PIL import Image
import random

from calibrate_fk.utils import (overlay_images,
                                read_data, check_model_path, check_data_path)

argument_parser = argparse.ArgumentParser()
argument_parser.add_argument("--model", "-m", help="calibrated robot model. Saved in the calibrated_urdf folder. ")
argument_parser.add_argument("--data", "-d", type=str, nargs='+', help="Data that you want to train the calibration on. Stored in data folder.")
argument_parser.add_argument("--camera-settings", "-c", help="Camera settings file. Default: camera_settings.json in the calibrated model folder", default=None)
argument_parser.add_argument("--no-generate-images", "-n", help="Pass this flag if you do not want to generate images again but only perform the overlay", action="store_true")
argument_parser.add_argument("--max-images", "-mi", help="Maximum number of images to overlay", default=20, type=int)
argument_parser.add_argument("--view", "-v", choices=["sockets", "window", "file"], default="sockets", help="sockets: look from behind the two sockets towards the robot. window: choose the view in a visualization window and save it in the camera settings file. file: use the view saved in the camera settings file, without opening a window")
argument_parser.add_argument("--elevation", "-e", help="Elevation of the camera above the horizontal in degrees, for --view sockets", default=5.0, type=float)
argument_parser.add_argument("--no-fit-view", help="For --view window and file: use the camera view exactly as saved, without zooming and centering it on all the configurations", action="store_true")
args = argument_parser.parse_args()

model = args.model
data = args.data
max_images = args.max_images
script_directory = os.path.abspath(__file__)
parent_directory = os.path.join(os.path.dirname(script_directory), os.path.pardir)
model_folder = parent_directory + "/calibrated_urdf/" + model
camera_setting_file = args.camera_settings or model_folder + "/camera_settings.json"
generate_images = not(args.no_generate_images)
fit_view = not(args.no_fit_view)
view = args.view

check_model_path(model_folder)

model_config = yaml.load(open(model_folder+ "/config.yaml", 'r'), Loader=yaml.FullLoader)
urdf_name = model_config["urdf"]
end_effector = model_config["end_effector"]

urdf_file = model_folder + "/" + urdf_name + ".urdf"

data_path = [None] * len(data)
for i, d in enumerate(data):
    data_path[i] = os.path.abspath(os.path.join(parent_directory, 'data', d))
check_data_path(data_path)

robot = yourdfpy.URDF.load(urdf_file)
dof = len(robot.actuated_joints)
q_show = [0, ] * dof
counter = 0
for joint in robot.robot.joints:
    if joint.type == "revolute":
        q_show[counter] = (joint.limit.lower + joint.limit.upper) / 2
        counter += 1


if view == "file" and not os.path.exists(camera_setting_file):
    print(f"Camera settings file {camera_setting_file} does not exist. Run with --view window first.")
    sys.exit(1)

if view in ("window", "file") and os.path.exists(camera_setting_file):
    with open(camera_setting_file, 'r') as f:
        saved_camera = json.load(f)

    robot.scene.camera_transform = saved_camera['transform']
    robot.scene.camera.fov = saved_camera['fov']



if generate_images:
    data= read_data(data_path)

    # iterated over the keys of data 
    for keys, joints in data.items():
        print(keys)

        os.makedirs(f"{model_folder}/images/{keys}", exist_ok=True)

        images = []
        #read the data in the first key in the dictionary

        q_hole_0 = joints[0]
        q_hole_1 = joints[1]
        q_show = np.zeros(robot.num_actuated_joints)
        configs = np.vstack([q_hole_0, q_hole_1])

        if view == "window":
            q_show[:len(q_hole_0[0])] = q_hole_0[0]
            robot.update_cfg(q_show)
            print("Move the view such that you can nicely see the end-effector. Press q in the visualization window to save the camera settings.")
            robot.show()
            saved_camera = {
                'transform': robot.scene.camera_transform.tolist(),
                'fov': robot.scene.camera.fov.tolist(),
            }

            print(f"Camera settings saved in: {camera_setting_file}")
            with open(camera_setting_file, 'w') as f:
                json.dump(saved_camera, f)
            rotation = robot.scene.camera_transform
        elif view == "file":
            rotation = robot.scene.camera_transform
        else:
            # look from behind the two sockets towards the robot base, with z up,
            # such that the sockets are in front of the robot
            sockets = []
            for q in configs:
                q_show[:len(q)] = q
                robot.update_cfg(q_show)
                sockets.append(robot.get_transform(end_effector)[:3, 3])
            direction = -np.mean(sockets, axis=0)
            direction[2] = 0.0
            direction /= np.linalg.norm(direction)
            elevation = np.radians(args.elevation)
            # the camera looks along its -z axis
            z_axis = -(np.cos(elevation) * direction - np.sin(elevation) * np.array([0.0, 0.0, 1.0]))
            x_axis = np.cross([0.0, 0.0, 1.0], z_axis)
            x_axis /= np.linalg.norm(x_axis)
            rotation = np.eye(4)
            rotation[:3, :3] = np.column_stack([x_axis, np.cross(z_axis, x_axis), z_axis])

        if view == "sockets" or fit_view:
            # keep the viewing direction, but zoom and center the camera
            # such that all the configurations fit in the square image
            corners = []
            for q in configs:
                q_show[:len(q)] = q
                robot.update_cfg(q_show)
                corners.extend(robot.scene.bounds_corners.values())
            fov = min(robot.scene.camera.fov)
            robot.scene.camera.fov = [fov, fov]
            robot.scene.camera_transform = robot.scene.camera.look_at(
                np.vstack(corners), rotation=rotation, pad=1.05)

        for j, q_data in enumerate([q_hole_0, q_hole_1]):
            for i, q in enumerate(q_data):
                print(f"Creating image for hole {j+1} of 2 and config {i+1}/{len(q_data)}")
                q_show[:len(q)]=q
                robot.update_cfg(q_show)
                img_bin = robot.scene.save_image((800, 800))
                img = Image.open(BytesIO(img_bin))

                img_array = np.array(img.convert("RGBA")).astype(np.float32)


                images.append(img_array)
                with open(f'{model_folder}/images/{keys}/hole_{j}_config_{i}.png', 'wb') as f:
                    f.write(img_bin)
        
    
for keys in [os.path.basename(p) for p in data_path]:
    # Open the input image and read the image data in the folder input_path and append it to the input_image list
    output_image = []
    input_path = f"{model_folder}/images/{keys}/"
    output_path = f"{model_folder}/overlay_"+ keys + ".png"
    all_files = os.listdir(input_path) if os.path.isdir(input_path) else []
    image_files = [f for f in all_files if f.endswith(('.png', '.jpg', '.jpeg'))]
    if not image_files:
        print(f"No images in {input_path}. Run without --no-generate-images first.")
        continue

    # Select 14 random images
    if len(image_files) >= max_images:
        selected_images = random.sample(image_files, max_images)
    else:
        selected_images = image_files  
    for filename in selected_images:
        with open(input_path + filename, 'rb') as f:
            input_image = f.read()

            # Remove the background using rembg
            output_image.append(input_image)

    overlay_images(output_image, output_path)
    print(f"Overlay saved in: {output_path}")







# if overlay and not data_path:
#     robot.update_cfg(q_show)
#     robot.show()
#     saved_camera = {
#         'transform': robot.scene.camera_transform.tolist(), 
#         'fov': robot.scene.camera.fov.tolist(),
#     }

#     print(f"Camera settings saved in: {camera_setting_file}")


