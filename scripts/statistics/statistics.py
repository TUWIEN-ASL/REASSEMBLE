#!/usr/bin/env python3
"""
Statistics computation for H5 files containing hierarchical and low-level action data.
This script analyzes h5 files and computes statistics on high-level actions, low-level actions,
completion steps, and success rates.
"""

import os
import pandas as pd
import numpy as np
import argparse
from REASSEMBLE.io import load_segment_info


def calc_stats(records, keep_lists=False):
    """
    Calculate statistics from action records loaded from H5 files. The script creates two separate tables. The first
    one analyzes low-level actions regarding success rate and duration. The second table has a histogram-like structure,
    describing the number of steps (low-level) necessary to perform a segment (high-level). Both tables are sorted
    alphabetically and have a total row in the bottom.
    
    Parameters:
    -----------
    records : dict
        Dictionary of records loaded from H5 files (filename -> record data). Can be easily created by our other script
        REASSEMBLE.io.load_segment_info(h5_dir).
    keep_lists : bool, optional
        Whether to retain the original lists of action durations in the output dataframes. (default: False)
        
    Returns:
    --------
    tuple
        Two dataframes: (action statistics, step statistics)
    """
    # Initialize dataframes for action and step statistics
    df_act = pd.DataFrame(columns=['HL action', 'Object', 'LL action', 'Total', 'Successful', 'length_list'])
    df_steps = pd.DataFrame(columns=['HL action', 'Object', 'n_steps_list'])

    # Process each H5 file record
    for h5_file, record in records.items():
        # Process each high-level action segment
        for hl_idx, hl_segment in enumerate(record):
            text = hl_segment['text']
            # Skip segments with no action
            if text == 'No action.':
                continue
                
            # Parse high-level action and object
            text = text.split(' ')
            hl_action = text[0]
            obj = ' '.join(text[1:]).replace('.', '')
            n_steps = len(hl_segment['low_level'])

            # Update steps dataframe
            # Search for existing row
            row = df_steps.loc[
                (df_steps['HL action'] == hl_action) &
                (df_steps['Object'] == obj)]

            if len(row):  # Add to existing row
                df_steps.loc[row.index[0], 'n_steps_list'].append(n_steps)
            else:  # Create new row
                row = pd.DataFrame([[hl_action, obj, [n_steps]]], columns=df_steps.columns)
                df_steps = pd.concat([df_steps, row], ignore_index=True)

            # Check for empty low-level steps
            if n_steps < 1:
                print(f'No low-level steps in {h5_file} index {hl_idx} ({hl_action} {obj}).')
                
            # Process each low-level action within the high-level action
            for ll_segment in hl_segment['low_level']:
                ll_action = ll_segment['text']
                length = ll_segment['end'] - ll_segment['start']
                flag = int(ll_segment['success'])

                # Update action statistics dataframe
                # Search for existing row
                row = df_act.loc[
                    (df_act['HL action'] == hl_action) &
                    (df_act['Object'] == obj) &
                    (df_act['LL action'] == ll_action)]

                if len(row):  # Add to existing row
                    row_idx = row.index[0]
                    df_act.loc[row_idx, 'Total'] += 1
                    df_act.loc[row_idx, 'Successful'] += flag
                    df_act.loc[row_idx, 'length_list'].append(length)
                else:  # Create new row
                    row = pd.DataFrame([[hl_action, obj, ll_action, 1, flag, [length]]], columns=df_act.columns)
                    df_act = pd.concat([df_act, row], ignore_index=True)

    # Convert lists to arrays for numerical processing
    df_act['length_array'] = np.array(df_act['length_list'])
    df_steps['n_steps_array'] = np.array(df_steps['n_steps_list'])

    # Add total row for summary statistics
    df_act.loc['total'] = df_act.sum()  # Sum numerical columns
    df_act.loc['total', 'HL action'] = 'total'
    df_act.loc['total', 'Object'] = 'all'
    df_act.loc['total', 'LL action'] = 'all'
    df_steps.loc['total'] = df_steps.sum()
    df_steps.loc['total', 'HL action'] = 'total'
    df_steps.loc['total', 'Object'] = 'all'

    # Calculate action statistics
    df_act['Success rate'] = df_act['Successful'] / df_act['Total'] * 100
    df_act['Total length'] = [np.sum(x) for x in df_act['length_array']]
    df_act['Avg length'] = df_act['Total length'] / df_act['Total']
    df_act['StDev length'] = [np.std(x) for x in df_act['length_array']]

    # Calculate step statistics
    df_steps['Min steps'] = [np.amin(x) for x in df_steps['n_steps_array']]
    df_steps['Max steps'] = [np.amax(x) for x in df_steps['n_steps_array']]
    df_steps['Avg steps'] = [np.mean(x) for x in df_steps['n_steps_array']]
    
    # Create histograms of step counts
    overall_max = np.amax(df_steps.loc['total', 'Max steps'])
    steps = ['No steps', '1 step'] + [f'{i} steps' for i in range(2, overall_max + 1)]
    bins = np.arange(0, overall_max + 2)
    df_steps[steps] = 0
    for row_idx, row in df_steps.iterrows():
        counts, _ = np.histogram(row['n_steps_array'], bins=bins)
        df_steps.loc[row_idx, steps] = counts

    # Clean up dataframes
    df_act.drop(columns='length_array', inplace=True)
    df_steps.drop(columns='n_steps_array', inplace=True)
    if not keep_lists:
        df_act.drop(columns='length_list', inplace=True)
        df_steps.drop(columns='n_steps_list', inplace=True)
        
    # Sort dataframes for better readability
    df_act = df_act.sort_values(by=['HL action', 'Object', 'LL action']).reset_index(drop=True)
    df_steps = df_steps.sort_values(by=['HL action', 'Object']).reset_index(drop=True)

    return df_act, df_steps


def parse_arguments():
    """
    Parse command line arguments.
    
    Returns:
    --------
    argparse.Namespace
        Parsed command line arguments
    """
    parser = argparse.ArgumentParser(
        description='Calculate statistics from H5 files containing action data.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument(
        '-i', '--input_dir',
        type=str,
        required=True,
        help='Directory containing H5 files to analyze'
    )
    
    parser.add_argument(
        '-oa', '--output_action',
        type=str,
        default='stats_action.csv',
        help='Output file path for action statistics'
    )
    
    parser.add_argument(
        '-os', '--output_steps',
        type=str,
        default='stats_steps.csv',
        help='Output file path for steps statistics'
    )
    
    parser.add_argument(
        '-k', '--keep_lists',
        action='store_true',
        help='Keep original lists in output dataframes'
    )
    
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Print detailed statistics to console'
    )
    
    return parser.parse_args()


if __name__ == '__main__':
    # Parse command line arguments
    args = parse_arguments()
    
    # Load H5 files from input directory
    print(f"Loading H5 files from: {args.input_dir}")
    records = load_segment_info(args.input_dir)
    print(f"Loaded {len(records)} H5 files")
    
    # Calculate statistics
    print("Calculating statistics...")
    stats_act, stats_steps = calc_stats(records, keep_lists=args.keep_lists)
    
    # Display statistics if verbose flag is set
    if args.verbose:
        print("\nAction Statistics:")
        print(stats_act.to_string())
        print("\nStep Statistics:")
        print(stats_steps.to_string())
    
    # Save statistics to output files
    output_action_path = os.path.join(args.input_dir, args.output_action)
    output_steps_path = os.path.join(args.input_dir, args.output_steps)
    
    print(f"Saving action statistics to: {output_action_path}")
    stats_act.to_csv(output_action_path)
    
    print(f"Saving step statistics to: {output_steps_path}")
    stats_steps.to_csv(output_steps_path)
    
    print("Done!")