#
#       examples.py
#       
#       Copyright 2010 Enrico Avventi <avventi@kth.se>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License version 2 as 
#       published by the Free Software Foundation.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
from numpy import array, ones
import slycot

def sb02md_example():
    A = array([ [0, 1],
                [0, 0]])
    Q = array([ [1, 0],
                [0, 2]])
    G = array([ [0, 0],
                [0, 1]])
    out = slycot.sb02md(2,A,G,Q,'C')
    print('--- Example for sb02md ---')      
    print('The solution X is')
    print(out[0])
    print('rcond =', out[1])
    
def sb03md_example():
    from numpy import zeros
    A = array([ [3, 1, 1],
                [1, 3, 0],
                [0, 0, 3]])
    C = array([ [25, 24, 15],
                [24, 32,  8],
                [15,  8, 40]])
    U = zeros((3,3))
    out = slycot.sb03md(3,C,A,U,'D')
    print('--- Example for sb03md ---')   
    print('The solution X is')
    print(out[0])
    print('scaling factor:', out[1])
    
def ab08nd_example():
    from numpy import zeros, size
    from scipy.linalg import eigvals
    A = array([ [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 3, 0, 0, 0],
                [0, 0, 0,-4, 0, 0],
                [0, 0, 0, 0,-1, 0],
                [0, 0, 0, 0, 0, 3]])
    B = array([ [0,-1],
                [-1,0],
                [1,-1],
                [0, 0],
                [0, 1],
                [-1,-1]])
    C = array([ [1, 0, 0, 1, 0, 0],
                [0, 1, 0, 1, 0, 1],
                [0, 0, 1, 0, 0, 1]])
    D = zeros((3,2))
    out = slycot.ab08nd(6,2,3,A,B,C,D)
    nu = out[0]
    print('--- Example for ab08nd ---')
    print('The finite invariant zeros are')
    print(eigvals(out[8][0:nu,0:nu],out[9][0:nu,0:nu]))

def ab13bd_example():
    A = array([[ -0.04165 ,  0.0000 ,  4.9200 ,  0.4920  , 0.0000   ,  0.0000 ,  0.0000 ],
               [ -5.2100  , -12.500 ,  0.0000 ,  0.0000  , 0.0000   ,  0.0000 ,  0.0000 ],
               [  0.0000  ,  3.3300 , -3.3300 ,  0.0000  , 0.0000   ,  0.0000 ,  0.0000 ],
               [  0.5450  ,  0.0000 ,  0.0000 ,  0.0000  , 0.0545   ,  0.0000 ,  0.0000 ],
               [  0.0000  ,  0.0000 ,  0.0000 , -0.49200 , 0.004165 ,  0.0000 ,  4.9200 ],
               [  0.0000  ,  0.0000 ,  0.0000 ,  0.0000  , 0.5210   , -12.500 ,  0.0000 ],
               [  0.0000  ,  0.0000 ,  0.0000 ,  0.0000  , 0.0000   ,  3.3300 , -3.3300 ]])
    B = array([[ 0.0000 , 0.0000 ],
               [ 12.500 , 0.0000 ],
               [ 0.0000 , 0.0000 ],
               [ 0.0000 , 0.0000 ],
               [ 0.0000 , 0.0000 ],
               [ 0.0000 , 12.500  ],
               [ 0.0000 , 0.0000 ]])
    C = array([[ 1.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 ],
               [ 0.0000 , 0.0000 , 0.0000 , 1.0000 , 0.0000 , 0.0000 , 0.0000 ],
               [ 0.0000 , 0.0000 , 0.0000 , 0.0000 , 1.0000 , 0.0000 , 0.0000 ]])
    D = array([[ 0.0000 , 0.0000 ],
               [ 0.0000 , 0.0000 ],
               [ 0.0000 , 0.0000 ]])
    out = slycot.ab13bd('C', 'L', 7, 2, 3, A, B, C, D, tol = 1e-10)
    print('--- Example for ab13bd ---')
    print('The L2-norm of the system is')
    print(out)

def ab13dd_example():
    from numpy import eye
    A = array([[    0 ,       1 ,  0 ,        0 ,  0 ,         0 ],
               [ -0.5 , -0.0002 ,  0 ,        0 ,  0 ,         0 ],
               [    0 ,       0 ,  0 ,        1 ,  0 ,         0 ],
               [    0 ,       0 , -1 , -0.00002 ,  0 ,         0 ],
               [    0 ,       0 ,  0 ,        0 ,  0 ,         1 ],
               [    0 ,       0 ,  0 ,        0 , -2 , -0.000002 ]])
    B = array([[ 1 ],
               [ 0 ],
               [ 1 ],
               [ 0 ],
               [ 1 ],
               [ 0 ]])
    C = array([[ 1 , 0 , 1 , 0 , 1 , 0 ]])
    D = array([[ 0 ]])
    out = slycot.ab13dd('C', 'I', 'N', 'D', 6, 1, 1, A, eye(6), B, C, D)
    print('--- Example for ab13dd ---')
    print('The L_infty norm of the system is')
    print(out[0])
    print('The peak frequency is')
    print(out[1])

def ab13ed_example():
    A = array([[ 0.1 , 1.0 , 0.0 , 0.0 , 0.0 ],
               [ 0.0 , 0.1 , 1.0 , 0.0 , 0.0 ],
               [ 0.0 , 0.0 , 0.1 , 1.0 , 0.0 ],
               [ 0.0 , 0.0 , 0.0 , 0.1 , 1.0 ],
               [ 0.0 , 0.0 , 0.0 , 0.0 , 0.1 ]])
    out = slycot.ab13ed(5, A, 9.)
    print('The lower bound of beta(A) is')
    print(out[0])
    print('The upper bound of beta(A) is')
    print(out[1])

def ab13fd_example():
    A = array([[  246.500 ,  242.500 ,  202.500 , -197.500 ],
               [ -252.500 , -248.500 , -207.500 ,  202.500 ],
               [ -302.500 , -297.500 , -248.500 ,  242.500 ],
               [ -307.500 , -302.500 , -252.500 ,  246.500 ]])
    out = slycot.ab13fd(4, A, 0.)
    print('The stability radius is')
    print(out[0])
    print('The minimizing omega is')
    print(out[1])
    
def mc01td_example():
    p = array([2, 0, 1, -1, 1])
    out = slycot.mc01td('C',4,p)
    print('--- Example for mc01td ...')
    if out[1]:
        print('The polynomial is stable')
    else:
        print('The polynomial has', out[2], 'unstable zeros')
        
def sb02od_example():
    from numpy import zeros, shape, dot, ones
    A = array([ [0, 1],
                [0, 0]])
    B = array([ [0],
                [1]])
    C = array([ [1, 0],
                [0, 1],
                [0, 0]])
    Q = dot(C.T,C)
    R = ones((1,1))
    out = slycot.sb02od(2,1,A,B,Q,R,'C')
    print('--- Example for sb02od ...')
    print('The solution X is')
    print(out[0])
    print('rcond =', out[1])
    
def tb03ad_example():
    A = array([ [1, 2, 0],
                [4,-1, 0],
                [0, 0, 1]])
    B = array([ [1, 0],
                [0, 0],
                [1, 0]])
    C = array([ [0, 1,-1],
                [0, 0, 1]])
    D = array([ [0, 1],
                [0, 0]])
    n = 3
    m = 1
    p = 2
    out = slycot.tb03ad(n,m,p,A,B,C,D,'R')
    #out = slycot.tb03ad_l(n,m,p,A,B,C,D)
    print('--- Example for tb03ad ...')
    print('The right polynomial representation of' )
    print('    W(z) = C(zI-A)^-1B + D')
    print('is the following:' )
    print('index', out[4])
    k_max = max(out[4]) + 1
    for k in range(0,k_max):
        print('P_%d =' %(k))
        print(out[5][0:m,0:m,k])
    for k in range(0,k_max):
        print('Q_%d =' %(k))
        print(out[6][0:m,0:p,k])
        
def tc04ad_example():
    from numpy import shape,zeros
    A = array([ [1, 2, 0],
                [4,-1, 0],
                [0, 0, 1]])
    B = array([ [1, 0],
                [0, 0],
                [1, 0]])
    C = array([ [0, 1,-1],
                [0, 0, 1]])
    D = array([ [0, 1],
                [0, 0]])
    n = 3
    m = 1
    p = 2
    out = slycot.tb03ad(n,m,p,A,B,C,D,'R')
    qcoeff = zeros((max(m,p),max(m,p),shape(out[6])[2]))
    qcoeff[0:shape(out[6])[0],0:shape(out[6])[1],0:shape(out[6])[1]]
    out2 = slycot.tc04ad(m,p,out[4],out[5][0:m,0:m,:],qcoeff,'R')
    print('--- Example for tb04ad ...')
    print('The system has a state space realization (A,B,C,D) where')
    print('A =')
    print(out2[1])
    print('B =')
    print(out2[2])
    print('C =')
    print(out2[3])
    print('D =')
    print(out2[4])

def tb01pd_example():
    A = array([[-1, 0],[0,-1]])
    B = ones((2,1))
    C = array([[0,1]])
    out = slycot.tb01pd(2, 1, 1, A, B, C)
    print('--- Example for tb01pd ...')
    print('Minimal realization for A, B, C')
    print('reduced order', out[-2])
    print(out)


def tb05ad_example():
    """
    Example of calculating the frequency response using tb05ad
    on a second-order system with a natural frequency of 10 rad/s
    and damping ratio of 1.05.
    """
    import numpy as np
    A = np.array([[0.0, 1.0],
                  [-100.0,   -20.1]])
    B = np.array([[0.],[100]])
    C = np.array([[1., 0.]])
    n = np.shape(A)[0]
    m = np.shape(B)[1]
    p = np.shape(C)[0]

    jw_s = [1j*11, 1j*15]
    at, bt, ct, g_1, hinvb,info = slycot.tb05ad(n, m, p, jw_s[0],
                                               A, B, C, job='NG')
    g_2, hinv2, info = slycot.tb05ad(n, m, p, jw_s[1], at, bt, ct, job='NH')
    print('--- Example for tb05ad...')
    print('Frequency response for (A, B, C)')
    print('-------------------------')
    print('Frequency  |     Response')
    print('%s        | %s '%(jw_s[0], g_1[0, 0]))
    print('%s        | %s '%(jw_s[1], g_2[0, 0]))


