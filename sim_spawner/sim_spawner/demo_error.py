from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
import omni
ext_manager = omni.kit.app.get_app().get_extension_manager()
simulation_app.update()

ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

simulation_app.close() # close Isaac Sim
