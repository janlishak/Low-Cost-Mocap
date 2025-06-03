import numpy as np

def main():
    # Prepare
    w, h = 512, 512
    fx, fy = 615, 618

    # Go
    fov_x = np.rad2deg(2 * np.arctan2(w, 2 * fx))
    fov_y = np.rad2deg(2 * np.arctan2(h, 2 * fy))

    print("Field of View (degrees):")
    print(f"  {fov_x = :.1f}\N{DEGREE SIGN}")
    print(f"  {fov_y = :.1f}\N{DEGREE SIGN}")

if __name__ == "__main__":
    main()