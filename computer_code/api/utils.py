import numpy as np

R_flip = np.diag([1, -1, -1])

def rotationToOpenCV(R_app: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix from app coordinates to OpenCV coordinates
    by flipping Y and Z axes.

    Args:
        R_app: np.ndarray of shape (3, 3) — rotation matrix in app coordinate frame.

    Returns:
        np.ndarray of shape (3, 3) — rotation matrix in OpenCV coordinate frame.
    """
    
    R_cv = R_flip @ R_app @ R_flip.T
    return R_cv


def translationToOpenCV(t_app: np.ndarray) -> np.ndarray:
    """
    Convert a 3D translation vector from app coordinates to OpenCV coordinates
    by flipping Y and Z axes.

    Args:
        t_app: np.ndarray of shape (3,) or (3, 1) — translation vector in app coordinate frame.

    Returns:
        np.ndarray of same shape — translation vector in OpenCV coordinate frame.
    """
    
    t_app = np.asarray(t_app, dtype=np.float32)

    if t_app.shape == (3,):
        return R_flip @ t_app
    elif t_app.shape == (3, 1):
        return (R_flip @ t_app)
    else:
        raise ValueError(f"Unsupported shape {t_app.shape}; expected (3,) or (3,1)")
    


if __name__ == "__main__":
    R = np.array([
        [ 0.76256755,  0.15331514, -0.62847848],
        [-0.02529311, -0.96370153, -0.26578115],
        [-0.64641395,  0.21857225, -0.7310097 ]
    ])
    t = np.ones((3, 1))

    print(f"R:\n{R}\nt:\n{t}")

    R = rotationToOpenCV(R)
    t = translationToOpenCV(t)

    print(f"R:\n{R}\nt:\n{t}")
