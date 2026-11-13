from setuptools import setup
from glob import glob
from os.path import isfile
setup(name='osp_mission', version='0.1.0', packages=['osp_mission'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/osp_mission']),
                  ('share/osp_mission', ['package.xml'])] +
                 [('share/osp_mission/' + d, [f for f in glob(d + '/*') if isfile(f)]) for d in ['launch', 'config', 'worlds', 'rviz', 'tools']],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      entry_points={'console_scripts': [
          'mission = osp_mission.mission:main', 'detector = osp_mission.detector:main',
          'record_video = osp_mission.record_video:main','startup_guard = osp_mission.startup_guard:main','calibrated_wheel_twist = osp_mission.calibrated_wheel_twist:main',
          'generate_assignment_scenarios = osp_mission.assignment_scenarios:main','generate_world = osp_mission.world:main','generate_scenarios = osp_mission.scenarios:main','train_policy = osp_mission.train:main','training_dashboard = osp_mission.dashboard:main','continual_train = osp_mission.continual:main','gazebo_eval = osp_mission.gazebo_eval:main']})
