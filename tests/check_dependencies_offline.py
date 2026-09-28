"""Import native modules and verify the bundled PCT cross-floor route."""
import sys
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_share_directory
root = Path(get_package_share_directory('pct_planner'))/'planner'
sys.path[:0] = [str(root), str(root/'scripts')]
from lib import a_star, ele_planner, traj_opt, py_map_manager
from config import Config
from planner_wrapper import TomogramPlanner

planner = TomogramPlanner(Config())
planner.loadTomogram('building2_9')
route = planner.plan(
    np.array([-5.5, 6.0, 0.5], dtype=np.float32),
    np.array([2.0, -3.0, 4.5], dtype=np.float32))
assert route is not None and 20 <= len(route) <= 200
assert np.linalg.norm(route[0, :2] - [-5.5, 6.0]) < 0.2
assert np.linalg.norm(route[-1, :2] - [2.0, -3.0]) < 0.2
assert route[-1, 2] - route[0, 2] > 3.5
print(f'PASS PCT native imports and {len(route)}-point cross-floor route')
if '--tomography' in sys.argv:
    import open3d
    import cupy
    print(f'PASS tomography imports: Open3D {open3d.__version__}, CuPy {cupy.__version__}')
else:
    print('SKIP optional Open3D/CuPy tomography dependencies; use --tomography to check them')
