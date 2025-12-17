import numpy as np
import matplotlib.pyplot as plt

from classes.Robot import Robot







def point_registration(A, B):
 
        assert A.shape == B.shape
        d, n = A.shape

        cA = A.mean(axis=1, keepdims=True)   # (d,1)
        cB = B.mean(axis=1, keepdims=True)   # (d,1)
        A0 = A - cA
        B0 = B - cB

    
        H = A0 @ B0.T

        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:             # reflection correction
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

    
        t = (cB - R @ cA).reshape(d)

        return R, t








def main():
    A = np.array(
        [
            [681.2, 526.9, 914.8],
            [542.3, 381.0, 876.5],
            [701.2, 466.3, 951.4],
            [598.4, 556.8, 876.9],
            [654.3, 489.0, 910.2],
        ]
    ).T

    B = np.array(
        [
            [110.1, 856.3, 917.8],
            [115.1, 654.9, 879.5],
            [167.1, 827.5, 954.4],
            [30.4, 818.8, 879.9],
            [117.9, 810.4, 913.2],
        ]
    ).T

    R, t = point_registration(A, B)

    print("Rotation matrix R:")
    print(np.round(R, 4))
    print(f"det(R): {np.linalg.det(R):.4f}")
    print("\nTranslation vector t:")
    print(np.round(t, 4))

    aligned = R @ A + t.reshape(-1, 1)
    errors = np.linalg.norm(aligned - B, axis=1)
    rmse = np.sqrt(np.mean(errors**2))



    print(f"\n Per-point alignment error (||R*A_i + t - B_i||): {errors}")
    print(f"\nMean alignment error: {errors.mean():.4f}")
    print(f"Max alignment error: {errors.max():.4f}")
    print(f"RMSE: {rmse}")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(A[0], A[1], A[2], marker="o", c="tab:blue", label="Source A")
    ax.scatter(B[0], B[1], B[2], marker="^", c="tab:green", label="Target B")
    ax.scatter(aligned[0], aligned[1], aligned[2], marker="x", c="tab:red", label="Transformed A")

    for i in range(A.shape[1]):
        ax.plot(
            [aligned[0, i], B[0, i]],
            [aligned[1, i], B[1, i]],
            [aligned[2, i], B[2, i]],
            c="gray",
            linestyle="--",
            linewidth=0.5,
        )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Point Registration Alignment")
    ax.legend()
    plt.tight_layout()
    plt.show()

    rmse = np.sqrt(np.mean(np.sum((aligned - B)**2, axis=1)))
    print("RMSE:", rmse)


if __name__ == "__main__":
    main()
