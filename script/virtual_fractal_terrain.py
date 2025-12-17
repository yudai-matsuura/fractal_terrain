import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


def plot_terrain_from_csv(csv_file, save_filename="terran_from_csv.png"):
    # Read csv file
    df = pd.read_csv(csv_file)
    x = df['x'].values
    y = df['y'].values
    z = df['z'].values

    # Offset
    x = x - np.min(x)
    y = y - np.min(y)

    # Create mesh
    x_unique = np.unique(x)
    y_unique = np.unique(y)
    X, Y = np.meshgrid(x_unique, y_unique)

    # Reshape z
    Z = z.reshape(len(y_unique), len(x_unique))

    # 3D plot
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    # Surface plot
    surf = ax.plot_surface(X, Y, Z, cmap='copper', edgecolor='none', alpha=0.9)
    # Add wireframe
    ax.plot_wireframe(X, Y, Z, color='black', linewidth=0.5, alpha=0.3)

    # axis label
    ax.set_xlabel('X (m)', fontsize=18, labelpad=15)
    ax.set_ylabel('Y (m)', fontsize=18, labelpad=15)
    ax.set_zlabel('Z (m)', fontsize=18, labelpad=15)
    ax.tick_params(axis='x', labelsize=16)
    ax.tick_params(axis='y', labelsize=16)
    ax.tick_params(axis='z', labelsize=14)
    ax.grid(False)

    # title
    ax.set_title("Terrain from CSV", fontsize=16)

    # arrange z range
    ax.set_xlim(0, np.max(X))
    ax.set_ylim(0, np.max(Y))
    z_min = np.min(Z)
    z_max = np.max(Z)
    ax.set_zlim(z_min, z_max)
    ticks = np.arange(-0.02, 0.02 + 0.02, 0.02)
    ax.set_zticks(ticks)

    # aspect ratio
    try:
        ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(Z)))
    except:
        pass

    # color bar
    # fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label='Height (m)')

    # save and display
    plt.savefig(save_filename, dpi=300)
    print(f"Saved image: {save_filename}")
    plt.show()


if __name__ == "__main__":
    csv_file = "fractal_terrain.csv"  # csv file name saved by C++
    plot_terrain_from_csv(csv_file)
