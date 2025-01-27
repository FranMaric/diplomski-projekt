import cv2
import rospy, tf
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import torch
from segment_anything import sam_model_registry, SamPredictor
import random
import sys
import math

np.set_printoptions(threshold=np.inf)

current_colored_image = None
current_depth_image = None
pose_publisher = None
tf_listener = None
predictor = None

cv_window_name = "Drone image"

def color_image_callback(data):
    global current_colored_image

    current_colored_image = data


def depth_image_callback(data):
    global current_depth_image

    current_depth_image = data


bridge = CvBridge()

camera_info = {'fx': 381.36246688113556, 'fy': 381.36246688113556, 'cx': 320.5, 'cy': 240.5}


def initialize_sam_model(pth_file):
    model_type = "vit_b"  # Use the SAM ViT-B model (smallest version)
    sam = sam_model_registry[model_type](checkpoint=pth_file)
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
    rospy.loginfo(f"Pixel ({x}, {y}) -> 3D Point ({X}, {Y}, {z})")
    return np.array([X, Y, Z])


# Mouse callback function to get selected points
def select_point(event, x, y, flags, param):
    image, points = param['image'], param['points']
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        color = (0, 0, 255) if len(points) == 1 else (255, 0, 0)  # Red for the first, Blue for others
        cv2.circle(image, (x, y), 5, color, -1)
        cv2.imshow(cv_window_name, image)
        rospy.loginfo(f"Point selected: ({x}, {y})")
        cv2.destroyAllWindows()


def compute_6dof_pose(points_3d, selected_points, fx, fy, cx, cy):
  
    #vraca 6dof poziciju i orijentaciju odabranih tocke 
    # PARAMETRI
    # points_3d su 3dimenzionalne verzije odabranih 2d tocaka
    # selected points su 2d tocke
    # fx i fy fokalne duljine 
    # cx i cy sredina slike 

    # VRACA
    #     dict: A dictionary containing:
    #         - 'success' (bool): Whether SolvePnP succeeded.
    #         - 'rvec' (numpy.ndarray): Rotation vector.
    #         - 'tvec' (numpy.ndarray): Translation vector.
    #         - 'position' (numpy.ndarray): Transformed 6DOF position (x, y, z) of the first point.
    #         - 'orientation' (numpy.ndarray): Orientation in degrees (rotation vector).
    
 
    object_points = np.array(points_3d, dtype=np.float32)
    image_points = np.array(selected_points, dtype=np.float32)
    
    # Define the camera intrinsic matrix
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros(5)  # Assuming no lens distortion

    # SolvePnP to compute rotation (rvec) and translation (tvec)
    success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    
    if success:
        print("Rotation vector (rvec):", rvec)
        print("Translation vector (tvec):", tvec)

        # Convert rotation vector to a rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # Transform the first 3D point into 6DOF coordinates
        first_point_3d = points_3d[0].reshape(3, 1)  # Convert to column vector
        transformed_point = np.dot(rotation_matrix, first_point_3d) + tvec
        print("Original 3D point (red):", first_point_3d.flatten())
        print("Transformed 6DOF Position (x, y, z):", transformed_point.flatten())

        # Include orientation (as Euler angles for interpretability)
        rvec_as_degrees = np.rad2deg(rvec).flatten()
        print("6DOF Orientation (Rotation Vector in degrees):", rvec_as_degrees)

        # Prepare the final 6DOF pose
        transformed_6dof = {
            "success": True,
            "rvec": rvec,
            "tvec": tvec,
            "position": transformed_point.flatten(),
            "orientation": rvec_as_degrees
        }
    else:
        print("solvePnP failed.")
        transformed_6dof = {
            "success": False,
            "rvec": None,
            "tvec": None,
            "position": None,
            "orientation": None
        }

    return transformed_6dof


def transform_from_camera_to_world_frame(pose_msg):
    try:
        pose_in_world = tf_listener.transformPose('world', pose_msg)
        pose_in_world.header.frame_id = 'world'
        rospy.loginfo("Pose transformed successfully.")

        return pose_in_world
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Error transforming pose: {e}")


def publish_6dof_pose(result):
    # SALJE RESULTAT DALJE LUKI NA OBRADU 
    if result["success"]:
        print("Transformed 6DOF Pose:")
        print("Position:", result["position"])
        print("Orientation (rotation vector in degrees):", result["orientation"])

        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'duckorange/camera'

        # Assign position (translation vector tvec)
        pose_msg.pose.position.x = result["position"][0]
        pose_msg.pose.position.y = result["position"][1]
        pose_msg.pose.position.z = result["position"][2]

        # Convert orientation (rotation vector in degrees) to quaternion
        rotation_vector = np.radians(result["orientation"])  # Convert degrees to radians
        angle = np.linalg.norm(rotation_vector)
        
        if angle > 1e-6:  # Avoid divide-by-zero for zero rotation
            axis = rotation_vector / angle
            quaternion = tf.transformations.quaternion_about_axis(angle, axis)
        else:
            # Identity rotation
            quaternion = [0, 0, 0, 1]

        # Assign quaternion to PoseStamped message
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        pose_msg = transform_from_camera_to_world_frame(pose_msg)

        pose_publisher.publish(pose_msg)
        print("Published 6DOF pose (position + rvec) to /drone/landing_pose")
    else:
        print("6DOF pose computation failed.")


def on_trigger_event_callback():
    if current_colored_image == None:
        rospy.logwarn("current_colored_image is None")
        return
    
    if current_depth_image == None:
        rospy.logwarn("current_depth_image is None")
        return
    
    image = bridge.imgmsg_to_cv2(current_colored_image, "bgr8")
    predictor.set_image(image)
    fx, fy = camera_info['fx'], camera_info['fy'] 
    cx, cy = image.shape[1] // 2, image.shape[0] // 2  # centar slike inace, ali treba probati i sa ovim vrijednostima iz camera_info da vidimo sto bolje radi
    display_image = image.copy()

    cv2.namedWindow(cv_window_name)
    cv2.imshow(cv_window_name, display_image)
    cv2.waitKey(1)

    # Set up mouse to select a landing point
    selected_point = []
    cv2.setMouseCallback(cv_window_name, select_point, {'image': display_image, 'points': selected_point})
    rospy.loginfo("Please select the surface")
    while len(selected_point) == 0:
        cv2.waitKey(1)
        continue

    best_mask, highlighted_image, mask_overlay = segment_surface_with_sam(image, selected_point, predictor)
    
    true_indices = np.argwhere(best_mask == 1)

    true_vals_with_row_index = true_indices[:, [1, 0]]  # Switch column and row order

    if len(true_vals_with_row_index) < 3:
        rospy.logwarn("Not enough points on the surface to select 3 random points.")
        return

    random_selected_points = random.sample(list(true_vals_with_row_index), 3) # Randomly select 3 points from mask
    rospy.loginfo(f"Randomly selected points: {random_selected_points}")

    # Combine the first selected point with the random points
    selected_points = selected_point + random_selected_points  # Add the random points to the selected points

    depth_image = bridge.imgmsg_to_cv2(current_depth_image, desired_encoding="passthrough")

    points_3d = [pixel_to_3d(p, depth_image, fx, fy, cx, cy) for p in selected_points]
    rospy.loginfo(f"3D Points: {points_3d}")

    try:
        result = compute_6dof_pose(points_3d, selected_points, fx, fy, cx, cy)

        # Check if the computation was successful
        if not result["success"]:
            rospy.loginfo("6DOF pose computation failed.")
            return

        rospy.loginfo("Transformed 6DOF Pose:")
        rospy.loginfo(f"Position: {result['position']}")
        rospy.loginfo(f"Orientation (degrees): {result['orientation']}")    

        publish_6dof_pose(result)
    except Exception as e:
        rospy.logerr(f"Failed to compute 6DOF pose: {e}")


# Main function
def main():
    global pose_publisher, tf_listener, predictor

    rospy.init_node("drone_video_processor", anonymous=True)

    pth_file = sys.argv[1]
    predictor = initialize_sam_model(pth_file)
    rospy.loginfo("Initialized SAM model")

    # Initialize ROS node
    rospy.Subscriber('/duckorange/camera/color/image_raw', Image, color_image_callback)
    rospy.Subscriber('/duckorange/camera/depth/image_raw', Image, depth_image_callback)
    tf_listener = tf.TransformListener()
    pose_publisher = rospy.Publisher("/drone/landing_pose", PoseStamped, queue_size=1)

    rospy.loginfo("Video processing node started")
    
    while not rospy.is_shutdown():        
        key = input("Enter F to capture an image:").strip().lower()
        if key == 'f':
            on_trigger_event_callback()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
