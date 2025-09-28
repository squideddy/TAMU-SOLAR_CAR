import numpy as np

# Mount type: 'A-ARM' or 'SHOCK'
mounttype = 'A-Arm'  # Change to 'SHOCK' for shock mount


inputs = np.array([[6255.272,	3392.495,	0.000],
[-13152.183,	764.162,	207.617],
[6627.199,	2591.850,	0.000],
[-12770.881,	3241.758,	-50.796],
[7438.080,	-655.969,	-324.562],
[7152.713,	-2105.017,	-517.960],
[10000.000, 0 , 0 ]])

mounttype_list = ['Shock','A-Arm','Shock','A-Arm','A-Arm','A-Arm','A-Arm']

# Bolt positions (relative to A at origin)
bolt1ARM = np.array([2.0,  .5, 0])
bolt2ARM = np.array([3.0,  0 , 0])
bolt3ARM = np.array([2.0, -1.35, 0])

bolt1Shock = np.array([2.04,  0.0, 0])
bolt2Shock = np.array([2.41, 1.39, 0])
bolt3Shock = np.array([1.04,  1.76, 0])


# Applied force at A (origin)
for i in range(0,len(inputs)):
    print("Case ", i+1)


    F_A = np.array(inputs[i,0:3]) # Fx, Fy, Fz, Mx, My, Mz
    F_A = np.append(F_A, [0, 0, 0])  # Add zero moments for bolts

    #print("Applied force at A:", F_A)

    # Function to create force + moment contribution of a bolt
    def force_moment_block(pos):
        # Identity matrix for Fx, Fy, Fz
        F = np.eye(3)
        # Cross product matrix [r]× to compute moment = r × F
        r = pos
        R = np.array([
            [0, -r[2], r[1]],
            [r[2], 0, -r[0]],
            [-r[1], r[0], 0]
        ])
        return np.vstack((F, R))

    # Build system matrix A (6×9)
    if mounttype == mounttype_list[i]:

        A = np.hstack((
            force_moment_block(bolt1ARM),
            force_moment_block(bolt2ARM),
            force_moment_block(bolt3ARM)
        ))
    else:
        A = np.hstack((
            force_moment_block(bolt1Shock),
            force_moment_block(bolt2Shock),
            force_moment_block(bolt3Shock)
        ))


    # Solve least-squares (since 6 eqs < 9 unknowns)
    R, residuals, rank, s = np.linalg.lstsq(A, -F_A, rcond=None)
    #print(R)
    totals = [sum(R[::3])+F_A[0], sum(R[1::3])+F_A[1], sum(R[2::3])+F_A[2]]   # Total force vector
    #print(totals)  # Check if total force is zero



    # Check to see if least squares solution is valid
    if not np.allclose(totals, [0, 0, 0], atol=1e-6):
        print("Warning: Total force is not zero, check calculations.")

    # Print results
    bolt_labels = ['Bolt 1', 'Bolt 2', 'Bolt 3'] # bolts labeled form top to bottom then left to right
    for i in range(3):
        Rx, Ry, Rz = R[3*i : 3*i+3]
        print(f"{bolt_labels[i]}: Rx = {Rx:.2f} N, Ry = {Ry:.2f} N, Rz = {Rz:.2f} N")


        