import cv2
import numpy as np
import torch
from fastsam import FastSAM, FastSAMPrompt  # Import from the FastSAM package
import time

# Initialize FastSAM model
def initialize_fast_sam_model():
    model_checkpoint = "/Users/josiphanak/Documents/EDUCATION/UNI/FER/MA/2.god/1.SEM/PROJEKT/code/fastsam/FastSAM/FastSAM-s.pt"  # Replace with the path to your FastSAM checkpoint
    device = "cpu"  # Use 'cuda' if you have a compatible GPU
    model = FastSAM(model_checkpoint)
    return model

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
    image_path = "/Users/josiphanak/Documents/EDUCATION/UNI/FER/MA/2.god/1.SEM/PROJEKT/rgb_image_0.jpg"
    image = cv2.imread(image_path)
    
    if image is None:
        print("Error: Could not load the image.")
        return
    start_time = time.time()
    # Initialize FastSAM model
    model = initialize_fast_sam_model()

    # Display the image for selecting points
    display_image = image.copy()
    selected_points = []
    cv2.imshow("Select Points for Segmentation", display_image)
    
    init_time = time.time()
    elapsed_time = init_time - start_time
    print(f"TIME taken for initialization: {elapsed_time:.2f} seconds.")
    cv2.setMouseCallback("Select Points for Segmentation", select_point, {'image': display_image, 'points': selected_points})
    
    print("Please select points to segment the slanted surface. Press 'q' when done.")
    while True:
        if (cv2.waitKey(1) & 0xFF == ord('q')) or len(selected_points) > 0:
            break

    if not selected_points:
        print("No points were selected. Exiting.")
        return

    start_time = time.time()
    # Convert selected points for FastSAM
    input_points = np.array(selected_points)
    print(f"Selected points: {input_points}")

    # Perform segmentation using FastSAM
    results = model(
        image_path,  # Provide the image file path
        retina_masks=True,  # Enable higher-resolution masks
        imgsz=1024,  # Adjust image size for inference
    )

    # Use FastSAMPrompt to apply point-based segmentation
    prompt_processor = FastSAMPrompt(image_path, results, device="cpu")
    mask = prompt_processor.point_prompt(points=input_points, pointlabel=[1,1,1])  # Get the mask for the points

    # Check the shape and adjust the mask
    mask = mask.squeeze()  # Remove singleton dimensions, if any
    mask = (mask > 0).astype(np.uint8)  # Ensure binary mask (values 0 or 1)
    
    # Overlay the mask on the image
    mask_overlay = (mask[:, :, None] * np.array([255, 0, 0])).astype(np.uint8)  # Highlight in blue
    mask_overlay = cv2.resize(mask_overlay, (image.shape[1], image.shape[0]))  # Resize mask to match original image

    highlighted_image = cv2.addWeighted(image, 0.6, mask_overlay, 0.4, 0)
    process_time = time.time()
    elapsed_time = process_time - start_time
    print(f"Time taken for processing: {elapsed_time:.2f} seconds.")
    # Display the segmented surface
    cv2.imshow("Segmented Surface", highlighted_image)
    print("Surface segmented. Press any key to exit.")
    cv2.waitKey(0)

    # Cleanup
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()