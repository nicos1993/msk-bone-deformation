import opensim as osim

# Load your modified model
model = osim.Model('./examples_Rajagopal2015/Rajagopal2015_TorsTib_l_Prox0Dist-30Deg.osim')
state = model.initSystem()

# Get the bodies
calcn_l = model.getBodySet().get("calcn_l")
calcn_r = model.getBodySet().get("calcn_r")

# Define a local point (e.g., the heel point in the calcaneus frame)
local_point = osim.Vec3(0, 0, 0) 

# Find the global position
glob_point_l = calcn_l.findStationLocationInGround(state, local_point)
glob_point_r = calcn_r.findStationLocationInGround(state, local_point)

print(f"Left Heel Height: {glob_point_l.get(1)}")
print(f"Right Heel Height: {glob_point_r.get(1)}")

# Check the difference
diff = abs(glob_point_l.get(1) - glob_point_r.get(1))
print(f"Vertical Mismatch: {diff*1000:.6f} mm")