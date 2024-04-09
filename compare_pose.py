import subprocess
import os
import logging
from pathlib import Path

def sanitize_filename(filename):
    """Sanitize a filename to contain only alphanumeric, hyphen, and underscore characters."""
    return ''.join(c for c in filename if c.isalnum() or c in ['-', '_'])

def process_pose(livox_pose_folder: Path, nebula_pose_dir: Path, logger: logging.Logger,ape_filename):
    """Process pose data for Livox and Nebula sensors."""
    livox_pose_base = livox_pose_folder.name
    valid_livox_pose = sanitize_filename(livox_pose_base)
    valid_livox_pose1 = valid_livox_pose + "-livox"
    valid_nebula_pose1 = valid_livox_pose + "-nebula"

    valid_nebula_pose = nebula_pose_dir / valid_livox_pose 
    logger.info("Current pose: %s", valid_livox_pose)

    livox_pose_file = livox_pose_folder / f"{valid_livox_pose1}.txt"
    nebula_pose_file = valid_nebula_pose / f"{valid_nebula_pose1}.txt"

    try:
        # Output evo_ape
        result = subprocess.run(["evo_ape", "tum", str(livox_pose_file), str(nebula_pose_file), "-a", "--t_max_diff", "0.05", "--t_offset", "-0.2"], capture_output=True, text=True)
        output = result.stdout
        lines = output.strip().split("\n")
     
        with open(ape_filename, "a") as file:
            file.write(valid_livox_pose + "\t")

        with open(ape_filename, "a") as file:
            for index, line in enumerate(lines[3:10]):
                if index != 5:  
                    values = line.split("\t")
                    file_content = values[1]
                    file.write(file_content + "\t")

            file.write("\n")

        # Save evo_traj trajectory comparison, in the parent directory
        os.chdir(nebula_pose_dir.parent)
        if not os.path.exists("traj_filter"):
            os.makedirs("traj_filter")
        plot_file_path = nebula_pose_dir.parent/"traj_filter"/valid_livox_pose

        # subprocess.run(["evo_ape", "tum", str(livox_pose_file), str(nebula_pose_file),
        #                 "-va", "-p", "--plot_mode=xy", "--t_max_diff", "0.05", "--save_plot", str(plot_file_path)])
        
        subprocess.run(["evo_ape", "tum", str(livox_pose_file), str(nebula_pose_file),
                        "-va", "--t_max_diff", "0.05", "--t_offset", "-0.2", "--save_plot", str(plot_file_path)])

    except subprocess.CalledProcessError as e:
        logger.error("An error occurred while processing pose %s: %s", valid_livox_pose, e)

def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    livox_pose_dir = Path("/home/qijie/Data/Measurements/corner_case/livox_pose")  # Livox pose folder path
    nebula_pose_dir = Path("/home/qijie/Data/Measurements/corner_case/nebula_pose2")  # Nebula pose folder path
    ape_filename = Path("/home/qijie/Data/Measurements/corner_case/ape_filter.txt") # path to save ape
    content = "filename\tmax\tmean\tmedian\tmin\trmse\tstd\n"

    with open(ape_filename, "w") as file:
        file.write(content)

    for livox_pose_folder in livox_pose_dir.iterdir():
        if livox_pose_folder.is_dir():
            process_pose(livox_pose_folder, nebula_pose_dir, logger, ape_filename)

if __name__ == "__main__":
    main()
