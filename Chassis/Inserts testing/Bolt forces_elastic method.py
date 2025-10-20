import numpy as np

Inputforce = np.array([[12770.881, 3241.758, -50.796]])
#Inputforce = np.array([[0, -3241.758, -50.796]])

# Bolt positions (relative to A at origin)
bolt1ARM = np.array([2.0,  .5, 0])
bolt2ARM = np.array([3.0,  0 , 0])
bolt3ARM = np.array([2.0, -1.35, 0])

bolt1Shock = np.array([2.04,  0.0, 0])
bolt2Shock = np.array([2.41, 1.39, 0])
bolt3Shock = np.array([1.04,  1.76, 0])

# find the centroid of bolts
Armcenter = np.array([(bolt1ARM[0]+bolt2ARM[0]+bolt3ARM[0])/3, (bolt1ARM[1]+bolt2ARM[1]+bolt3ARM[1])/3, 0])
# new bolt positions relative to centroid
bolt1ARM_LOA = bolt1ARM - Armcenter
bolt2ARM_LOA = bolt2ARM - Armcenter
bolt3ARM_LOA = bolt3ARM - Armcenter

forcelocation_LOA = np.zeros(3) - Armcenter

#forces on bolts from shear 
forces_bolt1 = np.array([0,0,0]) - Inputforce/3
forces_bolt2 = np.array([0,0,0]) - Inputforce/3
forces_bolt3 = np.array([0,0,0]) - Inputforce/3

# forces on bolts from moment

# moment at centroid caused by force
moment_at_centroid = np.cross(forcelocation_LOA, Inputforce)
print("Moment at centroid: ", moment_at_centroid)
#Reaction = M*C/ I

# diameter is the same for all bolts so I = sum of R^2
Radius1 = np.linalg.norm(bolt1ARM_LOA)
Radius2 = np.linalg.norm(bolt2ARM_LOA)
Radius3 = np.linalg.norm(bolt3ARM_LOA)
print("Radii: ", Radius1, Radius2, Radius3)


I = Radius1**2 + Radius2**2 + Radius3**2
reaction1 = moment_at_centroid * Radius1 / I
reaction2 = moment_at_centroid * Radius2 / I
reaction3 = moment_at_centroid * Radius3 / I
print("moment reactions: ", reaction1, reaction2, reaction3)

# find the percpendicular direction of each bolt to apply the reaction force correctly
dir1 = np.array([bolt1ARM_LOA[1], -bolt1ARM_LOA[0], 0])
dir1 = dir1 / np.linalg.norm(dir1)
dir2 = np.array([bolt2ARM_LOA[1], -bolt2ARM_LOA[0], 0])
dir2 = dir2 / np.linalg.norm(dir2)
dir3 = np.array([bolt3ARM_LOA[1], -bolt3ARM_LOA[0], 0])
dir3 = dir3 / np.linalg.norm(dir3)
print("Direction of forces:", dir1, dir2, dir3)

reaction1 = reaction1[0][2] * dir1
reaction2 = reaction2[0][2] * dir2
reaction3 = reaction3[0][2] * dir3

#add the shear forces to the moment reactions
reaction1 = reaction1 + forces_bolt1[0]
reaction2 = reaction2 + forces_bolt2[0]
reaction3 = reaction3 + forces_bolt3[0]

print("Reactions: ", reaction1, reaction2, reaction3)

#sum of all forces should be zero
totalforces = reaction1 + reaction2 + reaction3 + Inputforce[0]
print("Total forces (should be zero): ", totalforces)

#moment check
moment_check = np.cross(bolt1ARM, reaction1) + np.cross(bolt2ARM , reaction2) + np.cross(bolt3ARM, reaction3) + np.cross(forcelocation_LOA - forcelocation_LOA, Inputforce[0])
print("Moment check (should be zero) around Force loc: ", moment_check)

moment_check1 = np.cross(bolt1ARM_LOA, reaction1) + np.cross(bolt2ARM_LOA, reaction2) + np.cross(bolt3ARM_LOA, reaction3) + np.cross(forcelocation_LOA, Inputforce[0])
print("Moment check (should be zero) around origin: ", moment_check1)

# plot forces of bolts at the correct location and plot the input force
import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(111)
ax.quiver(bolt1ARM[0], bolt1ARM[1],  reaction1[0], reaction1[1],  color='r')
ax.quiver(bolt2ARM[0], bolt2ARM[1], reaction2[0], reaction2[1],  color='g')
ax.quiver(bolt3ARM[0], bolt3ARM[1],  reaction3[0], reaction3[1], color='b')
ax.quiver(forcelocation_LOA[0], forcelocation_LOA[1], Inputforce[0][0], Inputforce[0][1], color='k')
ax.set_xlim([-3, 4])
ax.set_ylim([-2, 2])

ax.set_xlabel('X')
ax.set_ylabel('Y')

plt.show()

