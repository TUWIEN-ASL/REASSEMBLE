"""
Radar Plot Generator for Action Analysis

This script processes HDF5 files containing action segment information and generates 
radar or bar charts to visualize success rates and other statistics by action type.

Usage:
    python radar_plot.py --input_dir /path/to/h5_files --output radar_plot.png --style radar
"""

import argparse
import os
import h5py
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, RegularPolygon
from matplotlib.path import Path
from matplotlib.projections import register_projection
from matplotlib.projections.polar import PolarAxes
from matplotlib.spines import Spine
from matplotlib.transforms import Affine2D


def parse_arguments():
    """
    Parse command line arguments for the script.
    
    Returns:
        argparse.Namespace: Parsed command-line arguments
    """
    parser = argparse.ArgumentParser(description='Generate radar or bar plots from HDF5 action data.')
    parser.add_argument('--input_dir', type=str, default='/srv/asl/NIST_full/data_final_final/',
                        help='Directory containing the HDF5 files')
    parser.add_argument('--output', type=str, default='gears_radar_plot.png',
                        help='Output filename for the generated plot')
    parser.add_argument('--style', type=str, choices=['radar', 'bar'], default='radar',
                        help='Plot style: radar or bar')
    parser.add_argument('--export_stats', action='store_true',
                        help='Export statistics to CSV file')
    parser.add_argument('--stats_file', type=str, default='action_stats.csv',
                        help='CSV filename for exported statistics')
    parser.add_argument('--figure_width', type=float, default=450/72,
                        help='Figure width in inches (default is 450/72)')
    
    return parser.parse_args()


def radar_factory(num_vars, frame='circle'):
    """
    Create a radar chart with a specified number of axes.

    This function creates a RadarAxes projection and registers it.
    Adapted from https://matplotlib.org/stable/gallery/specialty_plots/radar_chart.html

    Parameters:
        num_vars (int): Number of variables for radar chart.
        frame (str): Shape of frame surrounding Axes ('circle' or 'polygon').

    Returns:
        numpy.ndarray: Evenly-spaced axis angles.
    """
    # Calculate evenly-spaced axis angles
    theta = np.linspace(0, 2 * np.pi, num_vars, endpoint=False)

    class RadarTransform(PolarAxes.PolarTransform):
        def transform_path_non_affine(self, path):
            # Paths with non-unit interpolation steps correspond to gridlines,
            # in which case we force interpolation (to defeat PolarTransform's
            # autoconversion to circular arcs).
            if path._interpolation_steps > 1:
                path = path.interpolated(num_vars)
            return Path(self.transform(path.vertices), path.codes)

    class RadarAxes(PolarAxes):
        name = 'radar'
        PolarTransform = RadarTransform

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            # Rotate plot such that the first axis is at the top
            self.set_theta_zero_location('N')

        def fill(self, *args, closed=True, **kwargs):
            """Override fill so that line is closed by default"""
            return super().fill(closed=closed, *args, **kwargs)

        def plot(self, *args, **kwargs):
            """Override plot so that line is closed by default"""
            lines = super().plot(*args, **kwargs)
            for line in lines:
                self._close_line(line)

        def _close_line(self, line):
            """Close the given line by connecting the last point to the first point"""
            x, y = line.get_data()
            # FIXME: markers at x[0], y[0] get doubled-up
            if x[0] != x[-1]:
                x = np.append(x, x[0])
                y = np.append(y, y[0])
                line.set_data(x, y)

        def set_varlabels(self, labels):
            """Set the labels for each variable axis"""
            self.set_thetagrids(np.degrees(theta), labels)

        def _gen_axes_patch(self):
            """Generate the axes patch"""
            # The Axes patch must be centered at (0.5, 0.5) and of radius 0.5
            # in axes coordinates.
            if frame == 'circle':
                return Circle((0.5, 0.5), 0.5)
            elif frame == 'polygon':
                return RegularPolygon((0.5, 0.5), num_vars,
                                     radius=.5, edgecolor="k")
            else:
                raise ValueError(f"Unknown value for 'frame': {frame}")

        def _gen_axes_spines(self):
            """Generate the spines for the axes"""
            if frame == 'circle':
                return super()._gen_axes_spines()
            elif frame == 'polygon':
                # spine_type must be 'left'/'right'/'top'/'bottom'/'circle'.
                spine = Spine(axes=self,
                             spine_type='circle',
                             path=Path.unit_regular_polygon(num_vars))
                # unit_regular_polygon gives a polygon of radius 1 centered at
                # (0, 0) but we want a polygon of radius 0.5 centered at (0.5,
                # 0.5) in axes coordinates.
                spine.set_transform(Affine2D().scale(.5).translate(.5, .5)
                                   + self.transAxes)
                return {'polar': spine}
            else:
                raise ValueError(f"Unknown value for 'frame': {frame}")

    register_projection(RadarAxes)
    return theta


def calc_stats(h5_paths, h5_dir):
    """
    Calculate statistics from HDF5 files containing action segments.
    
    Parameters:
        h5_paths (list): List of HDF5 filenames to process
        h5_dir (str): Directory containing the HDF5 files
    
    Returns:
        pandas.DataFrame: DataFrame containing action statistics
    """
    # Initialize DataFrame with columns for statistics
    df = pd.DataFrame(columns=['Action', 'Total', 'Successful', 'length_list'])
    df.set_index('Action', inplace=True)

    # Process each HDF5 file
    for path in h5_paths:
        file_path = os.path.join(h5_dir, path)
        with h5py.File(file_path) as file:
            segments_info = file['segments_info']

            # Process each segment in the file
            for seg_id, seg_info in segments_info.items():
                # Extract action text and clean it
                action = str(seg_info['text'][...]).split("'")[1]
                action = action.strip()
                
                # Skip empty actions
                if action == 'No action.':
                    continue

                # Read segment information
                start = float(seg_info['start'][...])
                end = float(seg_info['end'][...])
                length = end - start
                success = bool(seg_info['success'][...])

                # If action not in data frame, add it
                if action not in df.index:
                    df.loc[action] = [0, 0, []]

                # Update statistics for this action
                df.loc[action, 'Successful'] += int(success)
                df.loc[action, 'Total'] += 1
                df.loc[action, 'length_list'].append(length)
    
    # Convert lists to numpy arrays for easier calculations
    df['length_array'] = df['length_list'].apply(np.array)

    # Calculate totals (sum of all actions)
    df.loc['total'] = df.sum(numeric_only=True)  # Sum numerical columns only
    df.at['total', 'length_list'] = []  # Initialize empty list for total row
    df.at['total', 'length_array'] = np.array([])  # Initialize empty array for total row

    # Calculate additional statistics
    df['Success rate'] = df['Successful'] / df['Total'] * 100
    df['Total length'] = df['length_array'].apply(np.sum)
    df['Avg length'] = df['Total length'] / df['Total']
    df['StDev length'] = df['length_array'].apply(np.std)

    # Clean up DataFrame by removing intermediate columns
    df.drop(columns=['length_array', 'length_list'], inplace=True)
    
    # Sort by index for better presentation
    df = df.sort_index()

    return df


def export_plot(df, style='bar', output_filename='plot.png', figure_width=450/72):
    """
    Generate and export a plot of action statistics.
    
    Parameters:
        df (pandas.DataFrame): DataFrame containing action statistics
        style (str): Plot style ('bar' or 'radar')
        output_filename (str): Filename for saving the plot
        figure_width (float): Width of the figure in inches
    """
    # Make a copy to avoid modifying the original DataFrame
    df = df.copy()
    
    # Remove total row for plotting
    if 'total' in df.index:
        df.drop(index='total', inplace=True)
    
    # Extract object and action_type from action strings
    # Assumes format like "action_type object"
    df['object'] = [action[action.find(' ') + 1:] for action in df.index]
    df['action_type'] = [action[:action.find(' ')] for action in df.index]
    
    # Get unique action types and objects
    action_types = sorted(set(df['action_type']))
    objects = set(df['object'])
    
    # Find maximum value for axis scaling
    axis_max = df['Total'].max()
    
    # Labels for action states
    action_states = ['failed', 'successful']
    
    # Configure plot appearance based on style
    if style == 'bar':
        colors = ['#c24e36', '#082a54']  # Color scheme for bar charts
        fig, axs = plt.subplots(2, 2, layout='constrained')
        legend_loc = 'outside lower right'
        theta = None
        
    elif style == 'radar':
        colors = ['#ff725c', '#1856a1']  # Color scheme for radar charts
        theta = radar_factory(len(objects), frame='polygon')
        fig, axs = plt.subplots(2, 2, subplot_kw={'projection': 'radar'})
        legend_loc = 'outside upper center'
        fig.subplots_adjust(wspace=0.25, hspace=0.20, top=0.85, bottom=0.05)
        
    else:
        raise ValueError(f"Unsupported plot style: {style}")
    
    # Create subplot for each action type
    for ax, action_type in zip(axs.flat, action_types):
        # Filter DataFrame for current action type
        sub_df = df[df['action_type'] == action_type]
        
        if style == 'bar':
            # Calculate failed actions
            failed = sub_df['Total'] - sub_df['Successful']
            
            # Create stacked bar chart
            ax.barh(sub_df['object'], failed, left=sub_df['Successful'], 
                    color=colors[0], label=action_states[0])
            ax.barh(sub_df['object'], sub_df['Successful'], 
                    color=colors[1], label=action_states[1])
            
            # Set title and labels
            ax.set_title(action_type.capitalize())
            ax.set_xlabel('Occurrences')
            ax.set_xlim([0, axis_max])
            
        elif style == 'radar':
            # Set radar chart title
            ax.set_title(
                action_type, weight='bold', size='medium', position=(0.5, 1.1),
                horizontalalignment='center', verticalalignment='center')
            
            # Create filled radar plots
            ax.fill(theta, sub_df['Total'], facecolor=colors[0])
            ax.fill(theta, sub_df['Successful'], facecolor=colors[1])
            
            # Set axis labels and limits
            ax.set_varlabels(sub_df['object'])
            ax.set_ylim([0, axis_max])
            ax.set_rgrids(np.linspace((axis_max-1)/3, axis_max-1, 3, dtype=int))
    
    # Add legend
    fig.legend(labels=action_states, loc=legend_loc, ncols=1, 
               framealpha=0, fontsize='small')
    
    # Set figure size
    fig.set_size_inches(1 * figure_width, 0.7 * figure_width)
    
    # Save figure
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    plt.close(fig)
    
    print(f"Plot saved to {output_filename}")


def main():
    """
    Main function to process HDF5 files and generate plots.
    """
    # Parse command-line arguments
    args = parse_arguments()
    
    # Validate input directory
    if not os.path.isdir(args.input_dir):
        print(f"Error: Input directory {args.input_dir} does not exist.")
        return
    
    # Find all HDF5 files in the directory
    h5_files = [file for file in os.listdir(args.input_dir) if file.endswith('.h5')]
    
    if not h5_files:
        print(f"No HDF5 files found in {args.input_dir}")
        return
    
    print(f"Found {len(h5_files)} HDF5 files to process.")
    
    # Calculate statistics from the files
    stats = calc_stats(h5_files, args.input_dir)
    
    # Export statistics to CSV if requested
    if args.export_stats:
        stats.to_csv(args.stats_file)
        print(f"Statistics exported to {args.stats_file}")
    
    # Print statistics summary
    print("\nAction Statistics Summary:")
    print("-" * 80)
    print(stats.to_string())
    print("-" * 80)
    
    # Generate and export plot
    export_plot(stats, style=args.style, output_filename=args.output, 
                figure_width=args.figure_width)


if __name__ == "__main__":
    main()