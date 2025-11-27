import re
import matplotlib.pyplot as plt

pxy_file = "robert.pxy"

xs, ys, zs = [], [], []

with open(pxy_file, "r") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        if line.startswith("XYZ-COORD-FILE"):
            continue  # skip header

        parts = line.split()
        # Expect: index x y z
        if len(parts) >= 4:
            try:
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
            except ValueError:
                continue
            xs.append(x)
            ys.append(y)
            zs.append(z)

print(f"Loaded {len(xs)} points from {pxy_file}")

# -------- 3D trajectory plot --------
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
ax.plot(xs, ys, zs, marker="o")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3D XYZ trajectory from .pxy")
ax.view_init(elev=25, azim=-60)  # nice angle
plt.tight_layout()
plt.savefig("pxy_3D_trajectory.png", dpi=300)

# -------- 2D projections for clarity --------
fig2, axes = plt.subplots(1, 3, figsize=(15, 4))

# Top view (plan): X vs Y
axes[0].plot(xs, ys, marker="o")
axes[0].set_xlabel("X")
axes[0].set_ylabel("Y")
axes[0].set_title("Top view (X–Y)")
axes[0].axis("equal")

# Side view: X vs Z
axes[1].plot(xs, zs, marker="o")
axes[1].set_xlabel("X")
axes[1].set_ylabel("Z")
axes[1].set_title("Side view (X–Z)")
axes[1].invert_yaxis()  # depth downwards, optional

# Front view: Y vs Z
axes[2].plot(ys, zs, marker="o")
axes[2].set_xlabel("Y")
axes[2].set_ylabel("Z")
axes[2].set_title("Front view (Y–Z)")
axes[2].invert_yaxis()

plt.tight_layout()
plt.savefig("pxy_projections.png", dpi=300)
plt.show()
