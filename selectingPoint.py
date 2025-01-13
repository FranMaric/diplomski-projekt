import cv2
import numpy as np

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
        cv2.imshow("Select 4 Points", image)
        print(f"Point selected: ({x}, {y})")
        if len(points) == 4:  # Stop after 4 points
            cv2.destroyAllWindows()

# Main program
def main():
    # Load the image and depth map
    image = cv2.imread("/Users/josiphanak/Documents/EDUCATION/UNI/FER/MA/2.god/1.SEM/PROJEKT/code/fran-podaci/images/color_image_19.jpeg")  # Replace with your image file path
    depth_data = np.load("/Users/josiphanak/Documents/EDUCATION/UNI/FER/MA/2.god/1.SEM/PROJEKT/code/fran-podaci/images/depth_image_distance_19.npy")  # Replace with your depth file path

    # Intrinsic camera parameters (fx, fy, cx, cy)
    fx, fy = 600, 600  # Focal lengths (example values, adjust to your camera)
    cx, cy = image.shape[1] // 2, image.shape[0] // 2  # Principal point

    # Copy of the image for visualization
    display_image = image.copy()

    # User selects 4 points
    selected_points = []
    cv2.imshow("Select 4 Points", display_image)
    cv2.setMouseCallback("Select 4 Points", select_point, {'image': display_image, 'points': selected_points})
    print("Please select 4 points.")
    while len(selected_points) < 4:
        cv2.waitKey(1)

    # Convert selected 2D points to 3D points
    points_3d = [pixel_to_3d(p, depth_data, fx, fy, cx, cy) for p in selected_points]
    print("3D Points:", points_3d)

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
