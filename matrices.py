import numpy as np
matrix_1 = np.zeros((10,10))
matrix_2 = np.zeros((10,10))
matrix_3 = np.zeros((10,10))

for i in range(0,10):
	for j in range(0,10):
		matrix_1[i][j] = j+1.1+i;

for i in range(0,10):
	for j in range(0,10):
		matrix_2[i][j] = j+1.1+i;


print(matrix_1)

print(matrix_2)

matrix_3 = np.zeros((10,10))
# print(matrix_3)
for i in range(0,10):
    for j in range(0,10): 
       for k in range(0,10):
             matrix_3[i][j]+=matrix_1[i][k]*matrix_2[k][j]
                                    
                                
                            
print(matrix_3);

#
''''
Resultado al rellenar las matrices

[[ 1.1  2.1  3.1  4.1  5.1  6.1  7.1  8.1  9.1 10.1]
 [ 2.1  3.1  4.1  5.1  6.1  7.1  8.1  9.1 10.1 11.1]
 [ 3.1  4.1  5.1  6.1  7.1  8.1  9.1 10.1 11.1 12.1]
 [ 4.1  5.1  6.1  7.1  8.1  9.1 10.1 11.1 12.1 13.1]
 [ 5.1  6.1  7.1  8.1  9.1 10.1 11.1 12.1 13.1 14.1]
 [ 6.1  7.1  8.1  9.1 10.1 11.1 12.1 13.1 14.1 15.1]
 [ 7.1  8.1  9.1 10.1 11.1 12.1 13.1 14.1 15.1 16.1]
 [ 8.1  9.1 10.1 11.1 12.1 13.1 14.1 15.1 16.1 17.1]
 [ 9.1 10.1 11.1 12.1 13.1 14.1 15.1 16.1 17.1 18.1]
 [10.1 11.1 12.1 13.1 14.1 15.1 16.1 17.1 18.1 19.1]]

 
[[ 1.1  2.1  3.1  4.1  5.1  6.1  7.1  8.1  9.1 10.1]
 [ 2.1  3.1  4.1  5.1  6.1  7.1  8.1  9.1 10.1 11.1]
 [ 3.1  4.1  5.1  6.1  7.1  8.1  9.1 10.1 11.1 12.1]
 [ 4.1  5.1  6.1  7.1  8.1  9.1 10.1 11.1 12.1 13.1]
 [ 5.1  6.1  7.1  8.1  9.1 10.1 11.1 12.1 13.1 14.1]
 [ 6.1  7.1  8.1  9.1 10.1 11.1 12.1 13.1 14.1 15.1]
 [ 7.1  8.1  9.1 10.1 11.1 12.1 13.1 14.1 15.1 16.1]
 [ 8.1  9.1 10.1 11.1 12.1 13.1 14.1 15.1 16.1 17.1]
 [ 9.1 10.1 11.1 12.1 13.1 14.1 15.1 16.1 17.1 18.1]
 [10.1 11.1 12.1 13.1 14.1 15.1 16.1 17.1 18.1 19.1]]

 '''
