#!/usr/bin/env python3
"""
Hierarchical Action Segmentation Flow Visualization

This script generates a Sankey diagram visualizing the flow between high-level and low-level actions
based on data from two CSV files. It calculates counts, creates informative labels, and produces
an interactive visualization with customizable colors and hover information.

Usage:
    python sankey_diagram.py --action-csv path/to/action_stats.csv --steps-csv path/to/steps_stats.csv [options]
"""

import argparse
import pandas as pd
import plotly.graph_objects as go
import colorsys
import os


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Generate a Sankey diagram for action segmentation flow.')
    
    # Required arguments
    parser.add_argument('--action-csv', type=str, required=True,
                        help='Path to the action statistics CSV file')
    parser.add_argument('--steps-csv', type=str, required=True,
                        help='Path to the steps statistics CSV file')
    
    # Optional arguments
    parser.add_argument('--output', type=str, default=None,
                        help='Output file path (supports .html, .png, .jpg, .svg, etc.)')
    parser.add_argument('--height', type=int, default=800,
                        help='Height of the diagram in pixels (default: 800)')
    parser.add_argument('--width', type=int, default=None,
                        help='Width of the diagram in pixels (default: auto)')
    parser.add_argument('--link-opacity', type=float, default=0.4,
                        help='Opacity of the links (0.0-1.0, default: 0.4)')
    parser.add_argument('--title', type=str, default="Hierarchical Action Segmentation Flow (with Counts)",
                        help='Title of the diagram')
    
    return parser.parse_args()


def hex_to_rgba(hex_color, opacity=0.6):
    """
    Convert hex color to rgba with specified opacity.
    
    Args:
        hex_color (str): Hex color code (e.g., '#FF5733')
        opacity (float): Opacity value between 0 and 1
        
    Returns:
        str: RGBA color string (e.g., 'rgba(255,87,51,0.6)')
    """
    hex_color = hex_color.lstrip('#')
    if len(hex_color) == 3:
        hex_color = ''.join([c*2 for c in hex_color])
    r, g, b = int(hex_color[0:2], 16), int(hex_color[2:4], 16), int(hex_color[4:6], 16)
    return f'rgba({r},{g},{b},{opacity})'


def get_semi_transparent_color(base_color, opacity=0.6):
    """
    Create a semi-transparent version of a color.
    Handles various color formats (rgba, hex, rgb).
    
    Args:
        base_color (str): Color in rgba, hex, or rgb format
        opacity (float): Opacity value between 0 and 1
        
    Returns:
        str: RGBA color string with the specified opacity
    """
    # If base_color is already rgba, parse it
    if base_color.startswith('rgba'):
        # Extract the numbers from the rgba string
        rgba_values = base_color.replace('rgba(', '').replace(')', '').split(',')
        r, g, b = int(rgba_values[0]), int(rgba_values[1]), int(rgba_values[2])
        return f'rgba({r},{g},{b},{opacity})'
    
    # If base_color is hex
    if base_color.startswith('#'):
        return hex_to_rgba(base_color, opacity)
    
    # If base_color is rgb
    if base_color.startswith('rgb'):
        rgb_values = base_color.replace('rgb(', '').replace(')', '').split(',')
        r, g, b = int(rgb_values[0]), int(rgb_values[1]), int(rgb_values[2])
        return f'rgba({r},{g},{b},{opacity})'
    
    # Default fallback
    return f'rgba(150,150,150,{opacity})'


def prepare_data(action_csv_path, steps_csv_path):
    """
    Read and prepare data from CSV files.
    
    Args:
        action_csv_path (str): Path to the action statistics CSV file
        steps_csv_path (str): Path to the steps statistics CSV file
        
    Returns:
        tuple: Contains various processed data structures needed for the Sankey diagram
    """
    # Read both CSV files
    print(f"Reading data from {action_csv_path} and {steps_csv_path}")
    df_action = pd.read_csv(action_csv_path)
    df_steps = pd.read_csv(steps_csv_path)

    # Filter out summary rows in the action data
    df_action = df_action[~df_action['HL action'].str.lower().str.contains('total', na=False)]

    # Get unique high-level and low-level actions
    hl_actions = sorted(df_action['HL action'].unique().tolist())
    ll_actions = sorted(df_action['LL action'].unique().tolist())

    # Calculate the correct high-level action counts from df_steps
    hl_totals = {}
    for hl_action in hl_actions:
        # Filter steps data for this high-level action
        hl_steps_data = df_steps[df_steps['HL action'] == hl_action]
        
        # Calculate total by summing all step columns for each object
        total_count = 0
        for _, row in hl_steps_data.iterrows():
            # Sum up all step columns (No steps, 1 step, 2 steps, etc.)
            step_cols = ['No steps', '1 step', '2 steps', '3 steps', '4 steps', 
                        '5 steps', '6 steps', '7 steps', '8 steps', '9 steps', '10 steps']
            step_sum = sum(row[col] for col in step_cols if pd.notna(row[col]))
            total_count += step_sum
        
        hl_totals[hl_action] = total_count

    # Convert to Series and sort by count for display order
    hl_totals = pd.Series(hl_totals)
    hl_totals = hl_totals.sort_values(ascending=False)
    hl_actions = hl_totals.index.tolist()

    # Calculate total counts for each low-level action
    ll_totals = df_action.groupby('LL action')['Total'].sum()

    # Create node labels with counts included
    node_labels = [f"{action} ({hl_totals[action]:,})" for action in hl_actions]
    node_labels += [f"{action} ({ll_totals[action]:,})" for action in ll_actions]

    # Gather data for creating links
    sources, targets, values, objects = [], [], [], []
    for _, row in df_action.iterrows():
        if pd.notna(row['HL action']) and pd.notna(row['LL action']) and pd.notna(row['Total']):
            source_idx = hl_actions.index(row['HL action'])
            target_idx = len(hl_actions) + ll_actions.index(row['LL action'])
            
            sources.append(source_idx)
            targets.append(target_idx)
            values.append(row['Total'])
            objects.append(row['Object'])

    return (df_action, df_steps, hl_actions, ll_actions, hl_totals, ll_totals, 
            node_labels, sources, targets, values, objects)


def group_links(sources, targets, values, objects):
    """
    Group links by source-target pairs to consolidate multiple objects.
    
    Args:
        sources (list): Source indices for each link
        targets (list): Target indices for each link
        values (list): Values (counts) for each link
        objects (list): Object names for each link
        
    Returns:
        tuple: Grouped sources, targets, values, and hover texts
    """
    # Group links by source-target pairs
    grouped_data = {}
    for i in range(len(sources)):
        key = f"{sources[i]}-{targets[i]}"
        if key not in grouped_data:
            grouped_data[key] = {
                'source': sources[i],
                'target': targets[i],
                'value': values[i],
                'objects': [objects[i]]
            }
        else:
            grouped_data[key]['value'] += values[i]
            if objects[i] not in grouped_data[key]['objects']:
                grouped_data[key]['objects'].append(objects[i])

    # Extract grouped data
    grouped_sources = []
    grouped_targets = []
    grouped_values = []
    grouped_hover_texts = []

    for item in grouped_data.values():
        grouped_sources.append(item['source'])
        grouped_targets.append(item['target'])
        grouped_values.append(item['value'])
        
        # Create detailed hover text
        objects_list = sorted(item['objects'], key=lambda x: x.lower())
        if len(objects_list) > 5:
            objects_text = ", ".join(objects_list[:5]) + f"... (+{len(objects_list)-5} more)"
        else:
            objects_text = ", ".join(objects_list)
        
        # Extract data for hover text
        source_idx = item['source']
        target_idx = item['target']
        
        # Create hover text with prominent count display
        hover_text = f"<b>Count: {item['value']:,}</b><br>Objects: {objects_text}"
        grouped_hover_texts.append(hover_text)
    
    return grouped_data, grouped_sources, grouped_targets, grouped_values, grouped_hover_texts


def create_sankey_diagram(node_labels, grouped_sources, grouped_targets, grouped_values, 
                          grouped_hover_texts, hl_actions, ll_actions, hl_totals, 
                          link_opacity=0.4, title="Hierarchical Action Segmentation Flow (with Counts)",
                          height=800, width=None):
    """
    Create a Sankey diagram visualization.
    
    Args:
        node_labels (list): Labels for each node
        grouped_sources (list): Source indices for each link
        grouped_targets (list): Target indices for each link
        grouped_values (list): Values (counts) for each link
        grouped_hover_texts (list): Hover texts for each link
        hl_actions (list): List of high-level actions
        ll_actions (list): List of low-level actions
        hl_totals (pd.Series): Totals for each high-level action
        link_opacity (float): Opacity for links
        title (str): Title of the diagram
        height (int): Height of the diagram in pixels
        width (int): Width of the diagram in pixels
        
    Returns:
        plotly.graph_objects.Figure: The Sankey diagram figure
    """
    # Set base colors for high-level actions (explicitly using rgba for better control)
    # Using very vibrant base colors with fully opaque nodes
    base_hl_colors = [
        'rgb(51, 161, 253)',   # blue
        'rgb(247, 152, 36)',   # orange
        'rgb(87, 167, 115)',   # green
        'rgb(219, 84, 97)',    # red
        'rgb(148, 103, 189)',  # purple
        'rgb(140, 86, 75)',    # brown
        'rgb(227, 119, 194)',  # pink
        'rgb(127, 127, 127)'   # gray
    ]

    # Colors for nodes - using fully opaque colors
    node_colors = [base_hl_colors[i % len(base_hl_colors)] for i in range(len(hl_actions))]
    node_colors += ["rgb(160, 160, 160)"] * len(ll_actions)  # Gray for all low-level actions

    # Colors for links - using semi-transparent colors
    link_colors = []
    for source in grouped_sources:
        base_color = base_hl_colors[source % len(base_hl_colors)]
        transparent_color = get_semi_transparent_color(base_color, link_opacity)
        link_colors.append(transparent_color)

    # Create the Sankey diagram with explicit opacity control
    fig = go.Figure(data=[go.Sankey(
        arrangement = "snap", 
        node = dict(
          pad = 20,
          thickness = 20,
          line = dict(color = "black", width = 0.5),
          label = node_labels,
          color = node_colors,  # Opaque node colors
          hovertemplate = "%{label}<extra></extra>"
        ),
        link = dict(
          source = grouped_sources,
          target = grouped_targets,
          value = grouped_values,
          hoverinfo = "none",
          hoverlabel = dict(
              bgcolor = "white", 
              font_size = 12,
              bordercolor = "black"
          ),
          customdata = grouped_hover_texts,
          hovertemplate = "%{customdata}<extra></extra>",
          color = link_colors,  # Semi-transparent link colors
          # Add label with counts to be displayed on the links
          label = [f"{val:,}" for val in grouped_values]
        )
    )])

    # Update layout with more customization
    layout_params = dict(
        title = dict(
            text = title,
            font = dict(size=22, color="black"),
            x = 0.5,
            y = 1.0
        ),
        font = dict(family="Arial", size=12),
        autosize = width is None,
        height = height,
        margin = dict(l=20, r=20, b=20, t=60),
        paper_bgcolor = 'white',
        plot_bgcolor = 'white'
    )
    
    # Add width if specified
    if width is not None:
        layout_params['width'] = width
        
    fig.update_layout(**layout_params)

    # Add subtitles for columns
    fig.add_annotation(
        x=0.05,
        y=1.05,
        xref="paper",
        yref="paper",
        text="High-level Actions (with Counts)",
        showarrow=False,
        font=dict(size=16, color="black"),
        align="left"
    )

    fig.add_annotation(
        x=0.95,
        y=1.05,
        xref="paper",
        yref="paper",
        text="Low-level Actions (with Counts)",
        showarrow=False,
        font=dict(size=16, color="black"),
        align="right"
    )

    # Add a color legend for high-level actions
    for i, action in enumerate(hl_actions):
        # Legend boxes use the opaque version of colors
        fig.add_shape(
            type="rect",
            x0=0.01, y0=0.99 - (i * 0.03), x1=0.03, y1=1.01 - (i * 0.03),
            line=dict(color="Black", width=1),
            fillcolor=base_hl_colors[i % len(base_hl_colors)],
            xref="paper",
            yref="paper"
        )
        
        # Show count in the legend as well
        fig.add_annotation(
            x=0.04,
            y=1.01 - (i * 0.03),
            xref="paper",
            yref="paper",
            text=f"{action} ({hl_totals[action]:,})",
            showarrow=False,
            font=dict(size=12),
            align="left",
            xanchor="left"
        )

    return fig


def main():
    """Main function to execute the script."""
    # Parse command line arguments
    args = parse_arguments()
    
    # Prepare data from CSV files
    (df_action, df_steps, hl_actions, ll_actions, hl_totals, ll_totals, 
     node_labels, sources, targets, values, objects) = prepare_data(args.action_csv, args.steps_csv)
    
    # Group links by source-target pairs
    grouped_data, grouped_sources, grouped_targets, grouped_values, grouped_hover_texts = group_links(
        sources, targets, values, objects)
    
    # Create the Sankey diagram
    fig = create_sankey_diagram(
        node_labels, grouped_sources, grouped_targets, grouped_values, grouped_hover_texts,
        hl_actions, ll_actions, hl_totals, args.link_opacity, args.title, args.height, args.width
    )
    
    # Save the figure if output path is provided
    if args.output:
        print(f"Saving diagram to {args.output}")
        extension = os.path.splitext(args.output)[1].lower()
        
        if extension == '.html':
            fig.write_html(args.output)
        else:
            # For image formats like .png, .jpg, .svg, etc.
            # Default to a higher resolution for static images
            width = args.width if args.width else 1600
            fig.write_image(args.output, width=width, height=args.height)
    else:
        # If no output path is provided, show the figure interactively
        print("Displaying diagram interactively (close browser window to exit)")
        fig.show()


if __name__ == "__main__":
    main()