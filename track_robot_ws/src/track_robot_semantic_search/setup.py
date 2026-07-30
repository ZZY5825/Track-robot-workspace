from glob import glob
from setuptools import setup


package_name = 'track_robot_semantic_search'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/schemas', glob('schemas/*.json')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='track-robot',
    maintainer_email='track-robot@example.com',
    description='Semantic-search Phase 0 contracts and replay tools.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'semantic_search_manifest = '
            'track_robot_semantic_search.manifest_cli:main',
            'semantic_search_localization_health = '
            'track_robot_semantic_search.localization_health_node:main',
            'semantic_search_evaluator = '
            'track_robot_semantic_search.evaluator_node:main',
            'semantic_search_compare_reports = '
            'track_robot_semantic_search.compare_reports:main',
            'semantic_search_select_text_model = '
            'track_robot_semantic_search.model_selection_cli:main',
            'semantic_search_perception = '
            'track_robot_semantic_search.perception_node:main',
            'semantic_search_yolo_world_perception = '
            'track_robot_semantic_search.yolo_world_perception_node:main',
            'semantic_search_query = '
            'track_robot_semantic_search.query_cli:main',
            'semantic_search_live_overlay = '
            'track_robot_semantic_search.live_overlay:main',
            'semantic_search_phase1_baselines = '
            'track_robot_semantic_search.phase1_baseline_cli:main',
            'semantic_search_phase2_evaluate = '
            'track_robot_semantic_search.phase2_evaluation_cli:main',
            'semantic_search_phase2_replay = '
            'track_robot_semantic_search.phase2_replay:main',
            'semantic_search_phase123_replay = '
            'track_robot_semantic_search.phase123_replay:main',
            'semantic_search_phase4_planner = '
            'track_robot_semantic_search.approach_planner_node:main',
            'semantic_search_phase4_validate = '
            'track_robot_semantic_search.phase04_validation:main',
            'semantic_search_phase04_live_validate = '
            'track_robot_semantic_search.phase04_live_validation:main',
            'semantic_search_phase4a_fixed_base = '
            'track_robot_semantic_search.fixed_base_session_node:main',
            'semantic_search_phase4a_selector = '
            'track_robot_semantic_search.phase4a_selector_node:main',
            'semantic_search_spatial_observation = '
            'track_robot_semantic_search.spatial_observation_node:main',
            'semantic_search_phase4a_advisor = '
            'track_robot_semantic_search.phase4a_advisor_node:main',
            'semantic_search_phase4a_validate = '
            'track_robot_semantic_search.phase4a_live_validation:main',
            'semantic_search_phase2_calibrate_task_threshold = '
            'track_robot_semantic_search.task_threshold_calibration_cli:main',
            'semantic_search_grounding_evaluate = '
            'track_robot_semantic_search.grounding_evaluation_cli:main',
            'semantic_search_grounding_select = '
            'track_robot_semantic_search.grounding_selection_cli:main',
        ],
    },
)
