from ur_control.utils import load_geometry_from_yaml, plot_geometry

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Visualize UR workspace geometry.')
    parser.add_argument('--config_path', type=str, default='src/ur_control/config/arm_parms.yaml')
    args = parser.parse_args()
    workspace, barrier, safe_zone, zmin, zmax = load_geometry_from_yaml(args.config_path)
    plot_geometry(workspace, barrier, safe_zone)