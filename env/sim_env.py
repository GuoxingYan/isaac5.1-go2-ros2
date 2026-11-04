import os
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
try:
    import isaacsim.storage.native as nucleus_utils
except ModuleNotFoundError:
    import isaacsim.core.utils.nucleus as nucleus_utils
from isaaclab.terrains import TerrainImporterCfg, TerrainImporter
from isaaclab.terrains import TerrainGeneratorCfg
from env.terrain_cfg import HfUniformDiscreteObstaclesTerrainCfg

def add_semantic_label():
    """Add semantic label to ground plane for Replicator.

    Note: In Isaac Sim 5.1, the ground prim path is /World/ground
    and Replicator's get.prims API may not be available in all configurations.
    """
    from isaacsim.core.utils.semantics import add_update_semantics
    for path, label in [("/World/ground", "floor"), ("/World/obstacleTerrain", "obstacle")]:
        prim = get_prim_at_path(path)
        if prim and prim.IsValid():
            add_update_semantics(prim, label)

def create_obstacle_sparse_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=100 ,
                obstacles_distance=2.0,
                border_width=5,
                platform_width=8.0,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain)
    add_semantic_label() 

def create_obstacle_medium_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=200 ,
                obstacles_distance=2.0,
                border_width=5,
                platform_width=8.0,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain)
    add_semantic_label() 


def create_obstacle_dense_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=400,
                obstacles_distance=2.0,
                border_width=5,
                platform_width=8.0,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain)
    add_semantic_label() 

LOCAL_ASSET_DIR = os.environ.get("GO2_WAREHOUSE_DIR", os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets", "Simple_Warehouse"))

def _load_warehouse(asset_file: str):
    add_semantic_label()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")

    # Try flattened version first (self-contained, no external refs)
    base_name = asset_file.replace(".usd", "")
    flat_path = f"{LOCAL_ASSET_DIR}/{base_name}_flat.usd"
    if os.path.exists(flat_path):
        import carb
        carb.log_info(f"Loading flattened warehouse from: {flat_path}")
        prim.GetReferences().AddReference(flat_path)
        return

    # Fallback: try local file
    local_path = f"{LOCAL_ASSET_DIR}/{asset_file}"
    if os.path.exists(local_path):
        import carb
        carb.log_info(f"Loading warehouse from local file: {local_path}")
        prim.GetReferences().AddReference(local_path)
    else:
        assets_root_path = nucleus_utils.get_assets_root_path()
        if assets_root_path:
            asset_path = assets_root_path + f"/Isaac/Environments/Simple_Warehouse/{asset_file}"
            prim.GetReferences().AddReference(asset_path)
        else:
            carb.log_error(f"Could not find warehouse asset: {asset_file}. "
                          "Try downloading assets or use obstacle environments instead.")

def create_warehouse_env():
    _load_warehouse("warehouse.usd")

def create_warehouse_forklifts_env():
    _load_warehouse("warehouse_with_forklifts.usd")

def create_warehouse_shelves_env():
    _load_warehouse("warehouse_multiple_shelves.usd")

def create_full_warehouse_env():
    _load_warehouse("full_warehouse.usd")

def create_hospital_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Hospital")
    prim = define_prim("/World/Hospital", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Hospital/hospital.usd"
    prim.GetReferences().AddReference(asset_path)

def create_office_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Office")
    prim = define_prim("/World/Office", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Office/office.usd"
    prim.GetReferences().AddReference(asset_path)