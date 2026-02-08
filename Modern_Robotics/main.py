from Tools import *
from FarwardKinematics import *
from Jacobian import *

#__________________________________________________________________________-


M = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 17],
    [0, 0, 0, 1]
])

w_vals = [
    [0, 0, 1], [1, 0, 0], [1, 0, 0], 
    [0, 0, 1], [1, 0, 0], [0, 0, 1]
]
L_vals = [
    [0, 0, -16], [2, 0, -12], [0, 0, -8], 
    [0, 0, -4], [0, 0, -3], [0, 0, -1]
]

L_valss = [
    [0, 0, 1], [2, 0, 5], [0, 0, 9], 
    [0, 0, 13], [0, 0, 14], [0, 0, 16]
]

thetas = np.array([
    np.radians(90), np.radians(-90), np.radians(90), 
    np.radians(-90), np.radians(90), np.radians(-90)
])


# Convert list of screws to a 6xN matrix (Transpose)
B_matrix = ConstructScrewlist(w_vals, L_vals)
B_matrixs = ConstructScrewlist(w_vals, L_valss)


# Call the function with the transposed matrix
Tfk = FKin(M, B_matrix, thetas, frame='Body')
Tfks = FKin(M, B_matrixs, thetas)

print(np.round(Tfk, 3))
print(np.round(Tfks, 3))

# print(GetFinalPose(Tfk))

# bj = np.round(Jacobian(B_matrix, thetas, frame='body'), 3)
# bs = np.round(Jacobian(B_matrixs, thetas), 3)

# print(np.round(Jacobian(B_matrix, thetas, frame='body'), 3))
# print(np.round(ConvertJacobianFrame(bs, Tfk, target_frame='body'),3))

# print(np.round(Jacobian(B_matrixs, thetas), 3))
# print(np.round(ConvertJacobianFrame(bj, Tfk, target_frame='space'),3))


'''__________________________________________________________________________'''

