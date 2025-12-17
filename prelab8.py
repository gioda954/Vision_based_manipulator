# prelab8.py
import cv2 as cv
import sys
from pathlib import Path

from classes.Robot import Robot  

def main():
  
    img_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("image_prelab8.jpg")
    if not img_path.exists():
        print(f"ERROR: cannot find '{img_path}'. Place the image next to this script or pass a path.")
        sys.exit(1)

    img = cv.imread(str(img_path), cv.IMREAD_COLOR)
    if img is None:
        print("ERROR: OpenCV failed to load the image.")
        sys.exit(1)

    bot = Robot()
    detections, vis = bot.detect_balls(img)

 
    if detections:
        print("Detected balls:")
        for d in detections:
            name = d["color"].capitalize()
            x, y = d["center"]
            print(f"- {name}: center=({x}, {y}), radius={d['radius']:.2f}px")

        centroids = [d["center"] for d in detections]
        radii = [round(d["radius"], 2) for d in detections]
        print(f"Centroid coordinates: {centroids}")
        print(f"Radii (px): {radii}")
    else:
        print("No balls detected.")

    # Visualize results
    try:
        cv.imshow("Original", img)
        cv.imshow("Processed (Detections)", vis)
        cv.waitKey(0)
        cv.destroyAllWindows()
    except cv.error:
        # Headless fallback
        out_path = Path("processed_prelab8.jpg")
        cv.imwrite(str(out_path), vis)
        print(f"GUI not available. Saved visualization to {out_path.resolve()}")

if __name__ == "__main__":
    main()
