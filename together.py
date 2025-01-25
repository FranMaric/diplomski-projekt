import cv2
import numpy as np
import torch
from segment_anything import sam_model_registry, SamPredictor
import random  # Import random module for selecting points randomly
np.set_printoptions(threshold=np.inf)
# Initialize SAM model
def initialize_sam_model():
    model_type = "vit_b"  # Use the SAM ViT-B model (smallest version)
    sam_checkpoint = "sam_vit_b_01ec64.pth"  # Path to the SAM ViT-B model checkpoint
    sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
    predictor = SamPredictor(sam)
    return predictor

# Function to perform segmentation using SAM
def segment_surface_with_sam(image, selected_points, predictor):
    input_points = np.array(selected_points)
    input_labels = np.ones(input_points.shape[0])  # All points are positive examples (label = 1)

    # Perform segmentation using SAM
    masks, scores, _ = predictor.predict(
        point_coords=input_points,
        point_labels=input_labels,
        # multimask_output=True  # Get multiple mask options
        multimask_output=False
    )

    # Select the best mask (highest score)
    best_mask = masks[np.argmax(scores)]
    # Overlay the mask on the image
    mask_overlay = (best_mask[:, :, None] * np.array([0, 0, 255])).astype(np.uint8)  # Highlight in blue
    highlighted_image = cv2.addWeighted(image, 0.6, mask_overlay, 0.4, 0)
    
    return best_mask, highlighted_image, mask_overlay

# Function to project a 2D pixel point into 3D using depth
def pixel_to_3d(pixel, depth_data, fx, fy, cx, cy):
    x, y = pixel
    z = depth_data[y, x]
    X = (x - cx) * z / fx
    Y = (y - cy) * z / fy
    Z = z
    print(f"Pixel ({x}, {y}) -> 3D Point ({X}, {Y}, {z})")
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
    # Load the image and depth map
    image = cv2.imread("/Users/josiphanak/Documents/EDUCATION/UNI/FER/MA/2.god/1.SEM/PROJEKT/rgb_image_0.jpg")
    depth_data = np.load("/Users/josiphanak/Documents/EDUCATION/UNI/FER/MA/2.god/1.SEM/PROJEKT/depth_image_0.npy")

    # Intrinsic camera parameters (fx, fy, cx, cy)
    fx, fy = 600, 600  # Focal lengths (example values, adjust to your camera)
    cx, cy = image.shape[1] // 2, image.shape[0] // 2  # Principal point

    # Initialize SAM model
    predictor = initialize_sam_model()
    predictor.set_image(image)
    
    # Display image for selecting points
    display_image = image.copy()  # Image copy for visualization
    selected_point = []
    cv2.imshow("Select 1 Points", display_image)
    cv2.setMouseCallback("Select 1 Points", select_point, {'image': display_image, 'points': selected_point})
    print("Please select 1 point.")
    while True:
        if (cv2.waitKey(1) & 0xFF == ord('q')) or len(selected_point) > 0:
            break

    # Perform surface segmentation using SAM
    best_mask, highlighted_image, mask_overlay = segment_surface_with_sam(image, selected_point, predictor)
    
    true_indices = np.argwhere(best_mask == 1)

    true_vals_with_row_index = true_indices[:, [1, 0]]  # Switch column and row order

  
    # Step 2: Randomly select 3 points from these surface points
    if len(true_vals_with_row_index) >= 3:
        random_selected_points = random.sample(list(true_vals_with_row_index), 3)
        print("Randomly selected points:", random_selected_points)
    else:
        print("Not enough points on the surface to select 3 random points.")

    # Combine the first selected point with the random points
    selected_points = selected_point + random_selected_points  # Add the random points to the selected points

    
    # Convert selected 2D points to 3D points
    points_3d = [pixel_to_3d(p, depth_data, fx, fy, cx, cy) for p in selected_points]
    print("3D Points:", points_3d)

    # Draw the points on the segmented surface (highlight first point in blue, random points in yellow)
    for i, point in enumerate(selected_points):
        if i == 0:
            # Draw first selected point in blue
            cv2.circle(highlighted_image, point, 4, (255, 0, 0), -1)  # Blue
        else:
            # Draw random selected points in yellow
            cv2.circle(highlighted_image, point, 4, (0, 255, 255), -1)  # Yellow

    # Show the final image with highlighted points
    cv2.imshow("Segmented Surface with Points", highlighted_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print("Surface segmented. Press any key to exit.")

    # SolvePnP to find the pose of the selected points
    object_points = np.array(points_3d, dtype=np.float32)  # 3D points
    image_points = np.array(selected_points, dtype=np.float32)  # Corresponding 2D points
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros(5)  # Assuming no lens distortion

    success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    if success:
        print("Rotation vector (rvec):", rvec)
        print("Translation vector (tvec):", tvec)

        # Convert rvec to a rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # Transform the first 3D point into 6DOF coordinates
        first_point_3d = points_3d[0].reshape(3, 1)  # Column vector
        transformed_point = np.dot(rotation_matrix, first_point_3d) + tvec
        print("Original 3D point (red):", first_point_3d.flatten())
        print("Transformed 6DOF Position (x, y, z):", transformed_point.flatten())

        # Include rotation (as Euler angles for better interpretability)
        rvec_as_degrees = np.rad2deg(rvec).flatten()
        print("6DOF Orientation (Rotation Vector in degrees):", rvec_as_degrees)

        # Final representation of the full 6DOF pose
        transformed_6dof = {
            "position": transformed_point.flatten(),
            "orientation": rvec_as_degrees
        }
        print("Transformed 6DOF Point:", transformed_6dof)

    else:
        print("solvePnP failed.")

if __name__ == "__main__":
    main()