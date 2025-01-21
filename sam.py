import cv2
import numpy as np
import torch
from segment_anything import sam_model_registry, SamPredictor

# Initialize SAM model
def initialize_sam_model():
    model_type = "vit_b"  # Use the SAM ViT-B model (smallest version)
    sam_checkpoint = "sam_vit_b_01ec64.pth"  # Path to the SAM ViT-B model checkpoint
    sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
    predictor = SamPredictor(sam)
    return predictor

# Mouse callback function to select points
def select_point(event, x, y, flags, param):
    image, points = param['image'], param['points']
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        cv2.circle(image, (x, y), 5, (0, 255, 0), -1)  # Highlight the selected point
        cv2.imshow("Select Points for Segmentation", image)
        print(f"Point selected: ({x}, {y})")

# Main program
def main():
    # Load the image
    image = cv2.imread("/Users/josiphanak/Documents/EDUCATION/UNI/FER/MA/2.god/1.SEM/PROJEKT/code/fran-podaci/images/color_image_22.jpeg")
    
    if image is None:
        print("Error: Could not load the image.")
        return

    # Initialize SAM model
    predictor = initialize_sam_model()
    predictor.set_image(image)

    # Display the image for selecting points
    display_image = image.copy()
    selected_points = []
    cv2.imshow("Select Points for Segmentation", display_image)
    cv2.setMouseCallback("Select Points for Segmentation", select_point, {'image': display_image, 'points': selected_points})
    
    print("Please select points to segment the slanted surface. Press 'q' when done.")
    while True:
        if (cv2.waitKey(1) & 0xFF == ord('q')) or len(selected_points) > 0:
            break

    if not selected_points:
        print("No points were selected. Exiting.")
        return

    # Convert selected points to numpy array for SAM
    input_points = np.array(selected_points)
    input_labels = np.ones(input_points.shape[0])  # All points are positive examples (label = 1)

    # Perform segmentation using SAM
    masks, scores, _ = predictor.predict(
        point_coords=input_points,
        point_labels=input_labels,
        multimask_output=True  # Get multiple mask options
    )

    # Select the best mask (highest score)
    best_mask = masks[np.argmax(scores)]

    # Overlay the mask on the image
    mask_overlay = (best_mask[:, :, None] * np.array([0, 0, 255])).astype(np.uint8)  # Highlight in blue
    highlighted_image = cv2.addWeighted(image, 0.6, mask_overlay, 0.4, 0)

    # Display the segmented surface
    cv2.imshow("Segmented Surface", highlighted_image)
    print("Surface segmented. Press any key to exit.")
    cv2.waitKey(0)

    # Cleanup
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()