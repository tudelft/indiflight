from indiflight_log_tools import IndiflightLog
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from scipy.spatial.transform import Rotation as R
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from flight_metrics import infer_learning_indices
from pyFlightPlotter import Quadrotor, Tailsitter


def plot_craft_3d(ax, craft, position, attitude_quat, surface_controls=None, geometry_scale=1.0, color='black', alpha=0.7):
    """
    Plot a 3D craft shape at the given position with the given attitude.
    
    Args:
        ax: 3D axis object
        craft: Craft3D object defining the vehicle geometry
        position: [x, y, z] position of the craft
        attitude_quat: [w, x, y, z] quaternion representing attitude
        color: color for the craft geometry
        alpha: transparency level
    """
    # Convert quaternion [w, x, y, z] to [x, y, z, w] format for scipy
    quat_scipy = np.array([attitude_quat[1], attitude_quat[2], attitude_quat[3], attitude_quat[0]])

    rot = R.from_quat(quat_scipy)

    # Plot filled body geometry with a translucent face.
    for body_segment in craft.geometry:
        scaled_segment = np.asarray(body_segment, dtype=float) * float(geometry_scale)
        translated_segment = rot.apply(scaled_segment) + position
        ax.add_collection3d(
            Poly3DCollection(
                [translated_segment],
                facecolors=color,
                edgecolors=color,
                linewidths=1.0,
                alpha=alpha,
            )
        )

    # Plot control surfaces as filled geometry, also translucent.
    if surface_controls is None:
        surface_controls = np.zeros(len(craft.surfaces), dtype=float)

    for surface, control in zip(craft.surfaces, surface_controls):
        tilt_xyz = np.asarray(surface["tilt_xyz"], dtype=float)
        tilt_axis = np.asarray(surface["tilt_axis"], dtype=float)
        tilt_rotation = R.from_rotvec(tilt_axis * float(control))

        for geo in surface["geometry"]:
            scaled_geo = np.asarray(geo, dtype=float) * float(geometry_scale)
            scaled_tilt_xyz = tilt_xyz * float(geometry_scale)
            geo_tilted = tilt_rotation.apply(scaled_geo - scaled_tilt_xyz) + scaled_tilt_xyz
            geo_world = rot.apply(geo_tilted) + position
            ax.add_collection3d(
                Poly3DCollection(
                    [geo_world],
                    facecolors=color,
                    edgecolors=color,
                    linewidths=0.8,
                    alpha=alpha * 0.55,
                )
            )


def color_for_index(idx, idx_start, idx_end):
    if idx < idx_start:
        return "black"
    if idx <= idx_end:
        return "red"
    return "green"
    




if __name__ == "__main__":
    parser = ArgumentParser(description="Plot 3D trajectory with vehicle attitude from a log file.",
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("logfile", type=str, help="Path to the log file.")
    parser.add_argument("--id", type=int, default=1, help="Log ID to use.")
    parser.add_argument("--resetTime", action="store_true", help="Reset time to start of the log.")
    parser.add_argument("--crop", required=False, nargs=2, metavar=("START", "END"), type=float,
                        help="Crop the log to the given time range (in seconds).")
    parser.add_argument("--auto-crop", action="store_true",
                        help="Automatically detect start/end times using learning detection (subtracts 0.5s from start, adds 1s to end).")
    parser.add_argument("--plot-every", type=float, default=1.0,
                        help="Plot craft position and attitude every N seconds.")
    parser.add_argument("--name", required=False, help="Name for the plot, used in title.")
    parser.add_argument("--craft-type", type=str, default="multirotor", 
                        choices=["quadrotor", "tailsitter"],
                        help="Type of craft for visualization.")
    parser.add_argument("--scale", type=float, default=0.5,
                        help="Scale factor for the craft visualization.")
    parser.add_argument("--output", type=str, default=None,
                        help="Path to write the figure as a PDF file.")
    parser.add_argument("--nr", type=int, default=2, help="Number of rotors.")
    parser.add_argument("--ns", type=int, default=2, help="Number of servos.")
    
    args = parser.parse_args()
    
    if args.name is None:
        args.name = args.logfile.split("/")[-1].split(".")[0]
    
    # Load the log
    log = IndiflightLog(args.logfile, logId=args.id, resetTime=args.resetTime)
    
    # Crop if requested
    if args.crop:
        print(f"Cropping data from {args.crop[0]}s to {args.crop[1]}s")
        log.data, _ = log.crop(args.crop[0], args.crop[1])
    elif args.auto_crop:
        # Automatically detect start and end times
        print("Auto-detecting start and end times...")
        idx_start, idx_end = infer_learning_indices(log.data, nr=args.nr, ns=args.ns)
        
        time = log.data['timeS'].to_numpy()
        start_time = time[idx_start] - 0.5
        end_time = time[idx_end] + 0.5
        
        # Clamp to log bounds
        start_time = max(start_time, time[0])
        end_time = min(end_time, time[-1])
        
        print(f"Auto-cropping from {start_time:.2f}s to {end_time:.2f}s")
        log.data, _ = log.crop(start_time, end_time)
    
    # Check if position and attitude data exists
    if 'pos[0]' not in log.data.columns:
        raise ValueError("Error: Position data (pos[0], pos[1], pos[2]) not found in log file.")
    
    if 'quat[0]' not in log.data.columns:
        raise ValueError("Error: Attitude data (quat[0], quat[1], quat[2], quat[3]) not found in log file.")
    
    # Extract time and data
    time = log.data['timeS'].to_numpy()
    position = log.data[['pos[0]', 'pos[1]', 'pos[2]']].to_numpy()
    quaternion = log.data[['quat[0]', 'quat[1]', 'quat[2]', 'quat[3]']].to_numpy()
    surface_feedback = None
    if 'servo_feedback[0]' in log.data.columns and 'servo_feedback[1]' in log.data.columns:
        surface_feedback = log.data[['servo_feedback[0]', 'servo_feedback[1]']].to_numpy()
    
    # Create figure and 3D axis
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot trajectory as a line
    ax.plot(position[:, 0], position[:, 1], position[:, 2], 
            'b-', linewidth=1, alpha=0.5, label='Trajectory')
    
    # Sample and plot craft at regular intervals
    plot_interval = args.plot_every
    start_time = time[0]
    end_time = time[-1]
    
    sample_times = np.arange(start_time, end_time + plot_interval, plot_interval)
    
    print(f"Plotting craft position and attitude every {plot_interval}s")
    print(f"Total flight time: {end_time - start_time:.2f}s")
    print(f"Number of craft visualizations: {len(sample_times)}")

    idx_start, idx_end = infer_learning_indices(log.data, nr=args.nr, ns=args.ns)
    
    # Create craft object
    if args.craft_type == "quadrotor":
        craft = Quadrotor()
    else:  # tailsitter
        craft = Tailsitter()
    
    for sample_time in sample_times:
        # Find closest index to this time
        idx = np.argmin(np.abs(time - sample_time))
        
        surface_controls = surface_feedback[idx] if surface_feedback is not None else None
        color = color_for_index(idx, idx_start, idx_end)
        plot_craft_3d(ax, craft, position[idx], quaternion[idx], surface_controls=surface_controls, geometry_scale=args.scale, color=color, alpha=0.6)

    # Set labels and title
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title(f"{args.name} -- 3D Trajectory\n{log.parameters.get('Firmware revision', 'Unknown firmware')}")
    
    # Set equal aspect ratio for better visualization
    # Get the current axis limits
    x_min, x_max = ax.get_xlim()
    y_min, y_max = ax.get_ylim()
    z_min, z_max = ax.get_zlim()
    
    # Calculate the maximum range
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min) / 2.0
    mid_x = (x_max + x_min) * 0.5
    mid_y = (y_max + y_min) * 0.5
    mid_z = (z_max + z_min) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Add dashed trajectory shadows onto the outer box planes.
    x_min, x_max = ax.get_xlim()
    y_min, y_max = ax.get_ylim()
    z_min, z_max = ax.get_zlim()
    x_plane = np.full_like(position[:, 0], x_min)
    y_plane = np.full_like(position[:, 1], y_max)
    z_plane = np.full_like(position[:, 2], z_max)
    ax.plot(x_plane, position[:, 1], position[:, 2], 'k--', linewidth=1.0, alpha=0.35, label='XZ shadow')
    ax.plot(position[:, 0], y_plane, position[:, 2], 'k--', linewidth=1.0, alpha=0.35, label='YZ shadow')
    ax.plot(position[:, 0], position[:, 1], z_plane, 'k--', linewidth=1.0, alpha=0.35, label='XY shadow')
    ax.invert_zaxis()
    
    ax.grid(True, alpha=0.3)
    ax.legend()

    # set camera
    ax.view_init(elev=21, azim=-63)
    
    plt.tight_layout()
    output_path = args.output if args.output is not None else f"{args.name}.pdf"
    fig.savefig(output_path, format="pdf", bbox_inches="tight")
    print(f"Saved PDF figure to {output_path}")
    plt.show()


