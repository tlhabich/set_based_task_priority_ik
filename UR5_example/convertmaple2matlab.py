# Generate symbolic matlab functions 
# Forwards kinematics and jacobians (analytical & geometric)
import os
import re

# DoF UR5
n=6;

os.chdir(os.getcwd()+r'.\maple_output')
if not os.path.isdir(r'.\matlab_fkine_transformation'):
    os.makedirs(r'.\matlab_fkine_transformation')
if not os.path.isdir(r'.\matlab_jacobian_analytic'):
    os.makedirs(r'.\matlab_jacobian_analytic')
if not os.path.isdir(r'.\matlab_jacobian_geometric'):
    os.makedirs(r'.\matlab_jacobian_geometric')

# generate fkine matlab function 
matlab_path = r'./matlab_fkine_transformation/trans_0_E_UR5.m'
maple_path = r'./fkine_transformation'
header_str = r'function T = trans_0_E_UR5(q, i)'
with open(matlab_path, mode='w') as matlab_func:
    matlab_func.write(header_str)
    matlab_func.write('\n')
for j in range(1, n+1):
    if j == 1:
        statement = f'if(i == {j})\n'
    elif j <= n:
        statement = f'elseif(i == {j})\n'
    with open(matlab_path, mode='a') as matlab_func:
        matlab_func.write(statement) 
    file_name = f'T0_{j}.m'
    tmp = ""
    with open(f'{maple_path}/{file_name}', mode='r') as ht_in:
    	tmp = tmp + ht_in.read().replace("unknown", "T")
    with open(matlab_path, mode='a') as matlab_func:
        matlab_func.write(tmp)
        matlab_func.write('\n')
statement = 'else\n'
with open(matlab_path, mode='a') as matlab_func:
        matlab_func.write(statement)
        matlab_func.write('error("index i out of range!")\n')
        matlab_func.write('end\nend\n')
print(f'Matlab function generated: {matlab_path}')  

# generate analytical jacobian matlab function
matlab_path = './matlab_jacobian_analytic/jac_a_UR5.m'
maple_path = './jacobian_analytic'
header_str = 'function J = jac_a_UR5(q)'
with open(matlab_path, mode='w') as matlab_func:
    matlab_func.write(header_str)
    matlab_func.write('\n') 
tmp = ""
file_name = f'J_a_{n}.m'
with open(f'{maple_path}/{file_name}', mode='r') as ht_in:
    tmp = tmp + ht_in.read().replace(r'%arctan', 'atan2').replace('unknown', 'J')
with open(matlab_path, mode='a') as matlab_func:
    matlab_func.write(tmp)
    matlab_func.write('\nend\n')
print(f'Matlab function generated: {matlab_path}')
 
# generate geomeric jacobian
matlab_path = f'./matlab_jacobian_geometric/jac_g_UR5.m'
maple_path = f'./jacobian_geometric'
header_str = f'function J = jac_g_UR5(q, i)'
with open(matlab_path, mode='w') as matlab_func:
    matlab_func.write(header_str)
    matlab_func.write('\n')
for i in range(1,n+1):
	tmp = ""
	file_name = f'J_geo_{n}_{i}.m'
	
	if i == 1:
		tmp = 'if i == 1' + '\n' + 'J = zeros(6,1);' + '\n'
	else:
		tmp = f'elseif i == {i}' + '\n' + f'J = zeros(6,{i});' + '\n'
	with open(f'{maple_path}/{file_name}', mode='r') as ht_in:
	    tmp = tmp + ht_in.read().replace(r'%arctan', 'atan2').replace('unknown', 'J')
	with open(matlab_path, mode='a') as matlab_func:
	    matlab_func.write(tmp)
	    matlab_func.write('\n') 
with open(matlab_path, mode='a') as matlab_func:
	matlab_func.write('\nend\nend\n')

print(f'Matlab function generated:: {matlab_path}')

