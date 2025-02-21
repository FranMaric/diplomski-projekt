import cv2
import numpy as np
import torch
from segment_anything import sam_model_registry, SamPredictor
import random  # Import random module for selecting points randomly
np.set_printoptions(threshold=np.inf)

# Camera intrinsic parameters -> insert values from the camera calibration
fx, fy = 381.36246688113556, 381.36246688113556  # Focal lengths
cx, cy = 320.5, 240.5  # Principal point

# Camera extrinsic parameters -> change for different setup 
yaw = -0.39269908169872414  # Given yaw in radians
R_cam = np.array([
    [np.cos(yaw), -np.sin(yaw), 0],
    [np.sin(yaw), np.cos(yaw),  0],
    [0,           0,            1]
])

#Translation vector -> change for setup
T_cam = np.array([[-6], [3], [4]])  


# Initialize SAM model with weights and return the predictor
def initialize_sam_model():
    model_type = "vit_b"  # Use the SAM ViT-B model (smallest version)
    sam_checkpoint = "/Users/josiphanak/Documents/EDUCATION/UNI/FER/MA/2.god/1.SEM/PROJEKT/code/sam_vit_b_01ec64.pth"  # Path to the SAM ViT-B model checkpoint
    sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
    predictor = SamPredictor(sam)
    return predictor

# Function to perform segmentation using SAM - returns the best mask and the highlighted image
def segment_surface_with_sam(image, selected_points, predictor):
    input_points = np.array(selected_points)
    input_labels = np.ones(input_points.shape[0])  # All points are positive examples (label = 1)
    masks, scores, _ = predictor.predict(
        point_coords=input_points,
        point_labels=input_labels,
        multimask_output=False
    )
    best_mask = masks[np.argmax(scores)]
    mask_overlay = (best_mask[:, :, None] * np.array([0, 0, 255])).astype(np.uint8)  # Highlight in blue
    highlighted_image = cv2.addWeighted(image, 0.6, mask_overlay, 0.4, 0)
    return best_mask, highlighted_image, mask_overlay

# Function to project a 2D pixel point into 3D using depth - pinhole camera model
def pixel_to_3d(pixel, depth_data):
    x, y = pixel
    z = depth_data[y, x]
    X = (x - cx) * z / fx
    Y = (y - cy) * z / fy
    Z = z
    return np.array([X, Y, Z])

# Mouse callback function to get selected points
def select_point(event, x, y, flags, param):
    image, points = param['image'], param['points']
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        color = (0, 0, 255) if len(points) == 1 else (255, 0, 0)  # Red for the first, Blue for others
        cv2.circle(image, (x, y), 5, color, -1)
        cv2.imshow("Select 1 Point", image)
        print(f"Point selected: ({x}, {y})")

# Main program
def main():
    image = cv2.imread("/Users/josiphanak/Documents/EDUCATION/UNI/FER/MA/2.god/1.SEM/PROJEKT/images/color_image_18.jpeg")
    depth_data = np.load("/Users/josiphanak/Documents/EDUCATION/UNI/FER/MA/2.god/1.SEM/PROJEKT/images/depth_image_distance_18.npy")

    predictor = initialize_sam_model()
    predictor.set_image(image)
    
    display_image = image.copy()
    selected_point = []
    cv2.imshow("Select 1 Point", display_image)
    cv2.setMouseCallback("Select 1 Point", select_point, {'image': display_image, 'points': selected_point})
    while True:
        if (cv2.waitKey(1) & 0xFF == ord('q')) or len(selected_point) > 0:
            break
    
    best_mask, highlighted_image, mask_overlay = segment_surface_with_sam(image, selected_point, predictor)
    true_indices = np.argwhere(best_mask == 1)
    true_vals_with_row_index = true_indices[:, [1, 0]]
    
    if len(true_vals_with_row_index) >= 3:
        random_selected_points = random.sample(list(true_vals_with_row_index), 3)
    else:
        print("Not enough points on the surface to select 3 random points.")
        return
    
    selected_points = selected_point + random_selected_points
    points_3d = [pixel_to_3d(p, depth_data) for p in selected_points]
    
    world_object_points = np.dot(R_cam, np.array(points_3d).T).T + T_cam.T
    image_points = np.array(selected_points, dtype=np.float32)
    
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros(5)
    success, rvec, tvec = cv2.solvePnP(world_object_points, image_points, camera_matrix, dist_coeffs)
    
    if success:
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        first_point_3d = points_3d[0].reshape(3, 1)
        transformed_point = np.dot(rotation_matrix, first_point_3d) + tvec
        rvec_as_degrees = np.rad2deg(rvec).flatten()
        
       
        v1 = points_3d[1] - points_3d[0]
        v2 = points_3d[2] - points_3d[0]
        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal)  # Normalize the normal vector
        angle_radians = np.arccos(normal[2])  # 2 index is the z component
        angle_degrees = np.degrees(angle_radians)
        angle_degrees = abs(90 - angle_degrees)
        transformed_6dof = {
            "position": transformed_point.flatten(),
            "orientation": rvec_as_degrees,
            "angle": angle_degrees
        }
        
        print("Transformed 6DOF Point:", transformed_6dof)
        print("Inclination angle:", (angle_degrees))
        print("Estimated Surface Normal Vector:", normal)
        print(rotation_matrix)
    else:
        print("solvePnP failed.")
    
    cv2.imshow("Segmented Surface with Points", highlighted_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
